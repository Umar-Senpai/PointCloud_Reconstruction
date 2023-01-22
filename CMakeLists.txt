cmake_minimum_required(VERSION 2.8)
project( Hello_Nanoflann )

#find_package(nanoflann)
find_package(Eigen3)
find_package( OpenCV REQUIRED )
find_package(PCL 1.2 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Main src/main.cpp)
target_include_directories(Main PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(Main ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(Reconstruction src/main_extra.cpp)
target_include_directories(Reconstruction PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(Reconstruction ${OpenCV_LIBS} ${PCL_LIBRARIES})

# READ MORE:
#   https://github.com/jlblancoc/nanoflann#11-obtaining-the-code
#   https://github.com/jlblancoc/nanoflann#17-use-in-your-project-via-cmake