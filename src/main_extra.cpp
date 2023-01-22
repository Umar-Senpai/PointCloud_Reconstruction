#include "../nanoflann.hpp"

#include <cstdlib>
#include <ctime>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include "boost.h"

using namespace Eigen;
using namespace std;
using namespace nanoflann;
using namespace std::chrono_literals;

// https://github.com/Gregjksmith/Iterative-Closest-Point
// https://github.com/nyakasko/icp_tricp/blob/main/src/main.cpp

const int SAMPLES_DIM = 15;

void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void addGaussianNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr& xyz_cloud, double standard_deviation)
{
  std::cout << "Adding Gaussian noise with mean 0.0 and standard deviation " << standard_deviation << std::endl;

  boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
  boost::normal_distribution<> nd (0, standard_deviation);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  for (size_t point_i = 0; point_i < xyz_cloud->points.size (); ++point_i)
  {
    xyz_cloud->points[point_i].x = xyz_cloud->points[point_i].x + static_cast<float> (var_nor ());
    xyz_cloud->points[point_i].y = xyz_cloud->points[point_i].y + static_cast<float> (var_nor ());
    xyz_cloud->points[point_i].z = xyz_cloud->points[point_i].z + static_cast<float> (var_nor ());
  }
}

pcl::visualization::PCLVisualizer::Ptr cloudsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

template <typename Der>
void generateRandomPointCloud(Eigen::MatrixBase<Der> &mat, const size_t N,
                              const size_t dim,
                              const typename Der::Scalar max_range = 10) {
  std::cout << "Generating " << N << " random points...";
  mat.resize(N, dim);
  for (size_t i = 0; i < N; i++)
    for (size_t d = 0; d < dim; d++)
      mat(i, d) = max_range * (rand() % 1000) / typename Der::Scalar(1000);
  std::cout << "done\n";
}

double calculateError(std::vector<float> distances) {
    double MSE = 0.;
    for (int i = 0; i < distances.size(); i++) {
        MSE += distances[i];
    }
    MSE /= distances.size();
    return MSE;
}

double calculateTricpError (int NPo, std::vector<int> sorted_indices, std::vector<float> distances) {
    double MSE = 0.;
    for (int i = 0; i < NPo; i++) {
        MSE += distances[sorted_indices[i]];
    }
    MSE /= NPo;
    return MSE;
}

Eigen::MatrixXf ICP(Eigen::MatrixXf source_pc, Eigen::MatrixXf transformed_pc, int MAX_I, Eigen::Matrix4f transform) {
  auto time_start = std::chrono::high_resolution_clock::now();
  using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
    Eigen::MatrixXf, 3, nanoflann::metric_L2>;
  Eigen::Matrix3f R_gt;
  R_gt << 
    transform(0,0), transform(0,1), transform(0,2),
    transform(1,0), transform(1,1), transform(1,2),
    transform(2,0), transform(2,1), transform(2,2);
  
  Eigen::Vector3f t_gt;
  t_gt << transform(0,3), transform(1,3), transform(2,3);

  my_kd_tree_t my_tree(3/*PC_type::ColsAtCompileTime */, std::cref(transformed_pc));
  my_tree.index->buildIndex();

  Eigen::Matrix3f R_total = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t_total = Eigen::Vector3f::Zero();

  double mean_error = 0.0;
  double prev_error = 0.0;
  double tolerance = 0.00001;
  int counter;
  for (counter = 0; counter < MAX_I; counter++) {
    // Find NN
    std::vector<size_t> indices;
    std::vector<float> distances;
    for (int i = 0; i < source_pc.rows(); i++) {
      // Query point:
      vector<float> query(3);
      for (int j = 0; j < 3; j++) {
          query[j] = source_pc(i, j);
      }
      // do a knn search
      nanoflann::KNNResultSet<float> result(1);
      size_t closest_index;
      float closest_sqdist;
      result.init(&closest_index, &closest_sqdist);
      my_tree.index->findNeighbors(result, &query[0], nanoflann::SearchParams());
      indices.push_back(closest_index);
      distances.push_back(closest_sqdist);
    }
    Eigen::MatrixXf new_transform = source_pc;
    for (int i = 0; i < source_pc.rows(); i++) {
            new_transform.row(i) = transformed_pc.row(indices[i]);
        }
    // transformed_pc = new_transform;
    Eigen::Vector3f source_mean = source_pc.colwise().mean();
    Eigen::Vector3f transform_mean = new_transform.colwise().mean();

    Eigen::MatrixXf centered_source = source_pc.rowwise() - source_mean.transpose();
    Eigen::MatrixXf centered_transform = new_transform.rowwise() - transform_mean.transpose();

    MatrixXf cov = (centered_source.transpose() * centered_transform)/float(centered_transform.rows()-1);

    // std::cout << cov << std::endl;
    Eigen::JacobiSVD<MatrixXf> svd(cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
    
    // std::cout << R << std::endl;

    Eigen::Vector3f t = transform_mean - R * source_mean;
    // std::cout << t << std::endl;
    R_total *= R;
    t_total += t;
    for(int i=0; i < source_pc.rows(); i++){
      source_pc.row(i) = (R * source_pc.row(i).transpose() + t).transpose();
    }

    mean_error = calculateError(distances); // Updating MSE
    // std::cout << "ERROR: " << mean_error << std::endl;
    if (abs(prev_error - mean_error) < tolerance) {
        break;
    }
    prev_error = mean_error;
    // cloud.push_back(pcl::PointXYZ(X, Y, Z));
    // std::cout << centered_source(0, 0) << centered_source(0, 1) << centered_source(0, 2) << std::endl;
  }
  auto time_finish = std::chrono::high_resolution_clock::now();
  
  std::chrono::duration<double> duration = (time_finish - time_start);
  std::cout << "--------------- ICP ---------------" << std::endl;
  std::cout << "Number of Iterations: " << counter << std::endl;
  std::cout << "Time = " << duration.count() << " s" << std::endl;
  std::cout << "MSE: " << mean_error << std::endl;
  std::cout << "Rotation matrix: " << endl << R_total << std::endl;
  std::cout << "Translation matrix: " << t_total << std::endl;
  std::cout << "Angular rotation error: " << std::acos(((R_total.transpose() * R_gt).trace() - 1) / 2) * 180.0 / M_PI << std::endl;
  std::cout << "Translation error: " << std::sqrt((t_total - t_gt).squaredNorm()) << std::endl;
  return source_pc;
}

Eigen::MatrixXf TrICP(Eigen::MatrixXf source_pc, Eigen::MatrixXf transformed_pc, int MAX_I, Eigen::Matrix4f transform) {
  auto time_start = std::chrono::high_resolution_clock::now();
  using my_kd_tree_t = nanoflann::KDTreeEigenMatrixAdaptor<
    Eigen::MatrixXf, 3, nanoflann::metric_L2>;

  Eigen::Matrix3f R_gt;
  R_gt << 
    transform(0,0), transform(0,1), transform(0,2),
    transform(1,0), transform(1,1), transform(1,2),
    transform(2,0), transform(2,1), transform(2,2);
  
  Eigen::Vector3f t_gt;
  t_gt << transform(0,3), transform(1,3), transform(2,3);

  my_kd_tree_t my_tree(3/*PC_type::ColsAtCompileTime */, std::cref(transformed_pc));
  my_tree.index->buildIndex();

  Eigen::Matrix3f R_total = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t_total = Eigen::Vector3f::Zero();
  float match_factor = 0.6;
  double mean_error = 0.0;
  double prev_error = 0.0;
  double tolerance = 0.00001;
  int counter;
  for (counter = 0; counter < MAX_I; counter++) {
    // Find NN
    std::vector<size_t> indices;
    std::vector<float> distances;
    for (int i = 0; i < source_pc.rows(); i++) {
      // Query point:
      vector<float> query(3);
      for (int j = 0; j < 3; j++) {
          query[j] = source_pc(i, j);
      }
      // do a knn search
      nanoflann::KNNResultSet<float> result(1);
      size_t closest_index;
      float closest_sqdist;
      result.init(&closest_index, &closest_sqdist);
      my_tree.index->findNeighbors(result, &query[0], nanoflann::SearchParams());
      indices.push_back(closest_index);
      distances.push_back(closest_sqdist);
    }

    std::vector<int> sorted_indices(indices.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](int A, int B) -> bool {
                    return distances[A] < distances[B];
                });

    int NPo = match_factor * double(sorted_indices.size());

    Eigen::MatrixXf new_transform(NPo, 3);
    Eigen::MatrixXf new_source(NPo, 3);

    for (int i = 0; i < NPo; i++) {
            new_source.row(i) = source_pc.row(sorted_indices[i]);
            new_transform.row(i) = transformed_pc.row(indices[sorted_indices[i]]);
        }

    // for (int i = 0; i < source_pc.rows(); i++) {
    //         new_transform.row(i) = transformed_pc.row(indices[i]);
    //     }
    // transformed_pc = new_transform;
    Eigen::Vector3f source_mean = new_source.colwise().mean();
    Eigen::Vector3f transform_mean = new_transform.colwise().mean();

    Eigen::MatrixXf centered_source = new_source.rowwise() - source_mean.transpose();
    Eigen::MatrixXf centered_transform = new_transform.rowwise() - transform_mean.transpose();

    MatrixXf cov = (centered_source.transpose() * centered_transform)/float(centered_transform.rows()-1);

    // std::cout << cov << std::endl;
    Eigen::JacobiSVD<MatrixXf> svd(cov, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
    // std::cout << R << std::endl;

    Eigen::Vector3f t = transform_mean - R * source_mean;
    // std::cout << t << std::endl;
    
    R_total *= R;
    t_total += t;

    for(int i=0; i < source_pc.rows(); i++){
      source_pc.row(i) = (R * source_pc.row(i).transpose() + t).transpose();
    }

    mean_error = calculateTricpError(NPo, sorted_indices, distances); // Updating MSE
    // std::cout << "ERROR: " << mean_error << std::endl;
    if (abs(prev_error - mean_error) < tolerance) {
        break;
    }
    prev_error = mean_error;
    // cloud.push_back(pcl::PointXYZ(X, Y, Z));
    // std::cout << centered_source(0, 0) << centered_source(0, 1) << centered_source(0, 2) << std::endl;
  }
  auto time_finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = (time_finish - time_start);
  std::cout << "--------------- TrICP ---------------" << std::endl;
  std::cout << "Number of Iterations: " << counter << std::endl;
  std::cout << "Time = " << duration.count() << " s" << std::endl;
  std::cout << "MSE: " << mean_error << std::endl;
  cout << "Rotation matrix: " << endl << R_total << endl;
  cout << "Translation matrix: " << endl << t_total << endl;
  cout << "Angular rotation error: " << std::acos(((R_total.transpose() * R_gt).trace() - 1) / 2) * 180.0 / M_PI << endl;
  cout << "Translation error: " << std::sqrt((t_total - t_gt).squaredNorm()) << endl;

  return source_pc;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertEigenToPCLCloud(Eigen::MatrixXf matrix) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for(int i=0; i < matrix.rows(); i++){
      cloud->push_back(pcl::PointXYZ(matrix(i, 0), matrix(i, 1), matrix(i, 2)));
    }
  cloud->width = cloud->size (); cloud->height = 1; cloud->is_dense = true;
  return cloud;
}

int main(int argc, char **argv) {
  
  // Randomize Seed
  if (argc < 8) {
    std::cerr << "Usage: " << argv[0] << " PC1 PC2 MAX_I noise_std rot_angle_deg trans_dist output" << std::endl;
    return 1;
  }

  // parse arguments
  std::string pc1 = argv[1];
  std::string pc2 = argv[2];
  int MAX_I = atoi(argv[3]);
  float noise_std = atof(argv[4]);
  float theta = atof(argv[5])* M_PI / 180.0;
  float trans_dist = atof(argv[6]);
  std::string output_file = argv[7];

  // rand seed
  srand(static_cast<unsigned int>(time(nullptr)));
  
  // load clouds from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  bool file_is_pcd = false;
  if (pc1.substr(pc1.find_last_of(".") + 1) == "pcd") {
      file_is_pcd = true;
  }

  if (file_is_pcd) {
    pcl::io::loadPCDFile(pc1, *source_cloud);
    pcl::io::loadPCDFile(pc2, *transformed_cloud);
  } else {
    pcl::io::loadPLYFile(pc1, *source_cloud);
    pcl::io::loadPLYFile(pc2, *transformed_cloud);
  }

  // Apply Transformation on one cloud
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform (0,0) = std::cos (theta);
  transform (0,1) = -sin(theta);
  transform (1,0) = sin (theta);
  transform (1,1) = std::cos (theta);
  // Define a translation of x meters on the x axis.
  transform (0,3) = trans_dist;

  // Executing the transformation
  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform);
  
  // Apply Noise
  addGaussianNoise(source_cloud, noise_std);
  Eigen::MatrixXf transposed_source_pc = source_cloud->getMatrixXfMap();
  Eigen::MatrixXf transposed_transformed_pc = transformed_cloud->getMatrixXfMap();

  removeRow(transposed_source_pc, 3);
  removeRow(transposed_transformed_pc, 3);

  Eigen::MatrixXf source_pc = transposed_source_pc.transpose();
  Eigen::MatrixXf transformed_pc = transposed_transformed_pc.transpose();
  Eigen::MatrixXf new_source_pc_tricp;
  std::cout << "Performing TrICP..." << std::endl;
  source_pc = TrICP(source_pc, transformed_pc, MAX_I, transform);

  // ICP
  // std::cout << "Performing ICP..." << std::endl;
  // Eigen::MatrixXf new_source_pc_icp = ICP(source_pc, transformed_pc, MAX_I, transform, outfileICP);
  
  for (int i=3; i < 9; i++){
    std::stringstream source_file;
	  source_file << "../office_chair/office_chair_1_" << i << ".ply";
    transformed_cloud->clear();
    pcl::io::loadPLYFile(source_file.str(), *transformed_cloud);
    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform);
    
    transposed_source_pc = transformed_cloud->getMatrixXfMap();

    removeRow(transposed_source_pc, 3);

    transformed_pc = transposed_source_pc.transpose();

    std::cout << "Performing TrICP..." << std::endl;
    new_source_pc_tricp = ICP(source_pc, transformed_pc, MAX_I, transform);

    source_pc = new_source_pc_tricp;
  
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr new_source_cloud_tricp (new pcl::PointCloud<pcl::PointXYZ> ());

  new_source_cloud_tricp = convertEigenToPCLCloud(new_source_pc_tricp);

  pcl::visualization::PCLVisualizer viewer_tricp ("Tr-ICP");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler_tricp (new_source_cloud_tricp, 255, 255, 255); // White
  viewer_tricp.addPointCloud (new_source_cloud_tricp, source_cloud_color_handler_tricp, "new_source_cloud_tricp");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler_tricp (transformed_cloud, 255, 255, 255); // Red
  viewer_tricp.addPointCloud (transformed_cloud, transformed_cloud_color_handler_tricp, "transformed_cloud");

  viewer_tricp.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "new_source_cloud_tricp");
  viewer_tricp.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  pcl::visualization::PCLVisualizer::Ptr viewer1;
  // BUG in my environment? The last window closes so added a 4th arbitrary window
  viewer1 = cloudsVis(source_cloud);


  // pcl::visualization::PCLVisualizer::Ptr viewer;
  // viewer = cloudsVis(cloud);
  std::cout << PCL_VERSION_PRETTY << std::endl;
  while (!viewer1->wasStopped ())
  {
    viewer1->spinOnce();
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}