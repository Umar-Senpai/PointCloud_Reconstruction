# PointCloud_Reconstruction
```
./Main PC1 PC2 MAX_I noise_std rot_angle_deg trans_dist output
```

```
./Main ../data/horse0.pcd ../data/horse1.pcd 500 1.0 20 10.0 ../results/fountain_1_20_10
```
## Wolf
Parameters | Ground Truth | ICP | TrICP
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
noise: 0<br>rot: 10&deg;<br>x-trans: 50m|![Screenshot from 2023-01-22 22-57-57](https://user-images.githubusercontent.com/31202659/213942541-5c3df420-5c70-48ac-b99d-0158750eb304.png) |  ![Screenshot from 2023-01-22 22-58-36](https://user-images.githubusercontent.com/31202659/213942552-f1619f79-9893-4d90-908b-0fa40887e9ab.png) | ![Screenshot from 2023-01-22 22-58-43](https://user-images.githubusercontent.com/31202659/213942573-28c425c9-01e4-46a6-bd81-d974364a27dd.png)
noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|![Screenshot from 2023-01-22 23-24-41](https://user-images.githubusercontent.com/31202659/213943668-eeada865-cc5b-42ac-9756-e914cdb64b31.png)|![Screenshot from 2023-01-22 23-24-56](https://user-images.githubusercontent.com/31202659/213943673-6901766f-819b-4409-8a7b-d79487ac0027.png)|![Screenshot from 2023-01-22 23-25-17](https://user-images.githubusercontent.com/31202659/213943685-9a812443-8ee1-42e2-9a3d-2fe9f126eb26.png)
noise: 0<br>rot: 20&deg;<br>x-trans: 0m|![Screenshot from 2023-01-22 23-31-20](https://user-images.githubusercontent.com/31202659/213943915-1a6d3bd1-5f0d-49fa-99ef-ebcb693e208d.png)|![Screenshot from 2023-01-22 23-31-44](https://user-images.githubusercontent.com/31202659/213943916-67ca52c3-f5db-44db-af4c-e4e50162461d.png)|![Screenshot from 2023-01-22 23-32-00](https://user-images.githubusercontent.com/31202659/213943919-2e5fed4d-93ca-4792-851d-90fd4479007b.png)


Algorithm | Parameters | # Iterations | Time Taken (s) | Rotational Error (Deg) | Translational Error | MSE
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
ICP|noise: 0<br>rot: 10&deg;<br>x-trans: 50m|22|0.293366|1.65682|7.59378|7.20658
TrICP|noise: 0<br>rot: 10&deg;<br>x-trans: 50m|30|0.46121|1.27165|7.24762|5.8861e-09
ICP|noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|26|0.35903|1.67561|13.7599|8.454
TrICP|noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|51|0.733307|1.34639|16.4488|0.964213
ICP|noise: 0<br>rot: 20&deg;<br>x-trans: 0m|19|0.251586|1.55872|1.41798|7.20665
TrICP|noise: 0<br>rot: 20&deg;<br>x-trans: 0m|30|0.419888|0.181308|1.02054|1.05461e-08

## Fountain
Parameters | Ground Truth | ICP | TrICP
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
noise: 0<br>rot: 10&deg;<br>x-trans: 10m|![Screenshot from 2023-01-23 00-11-17](https://user-images.githubusercontent.com/31202659/213945476-7d13e8d6-7b27-4552-94a2-4b0b3774845c.png)|![Screenshot from 2023-01-23 00-11-25](https://user-images.githubusercontent.com/31202659/213945481-8f83a84f-1d04-470b-b55d-d9238f7c717b.png)|![Screenshot from 2023-01-23 00-11-34](https://user-images.githubusercontent.com/31202659/213945484-ec9a644d-494f-46cd-a9b1-6d3d2e6e9fd9.png)
noise: 0<br>rot: 10&deg;<br>x-trans: 2m|![Screenshot from 2023-01-23 00-13-18](https://user-images.githubusercontent.com/31202659/213945585-d6d78d5f-56ad-42a6-b1cf-575930684d67.png)|![Screenshot from 2023-01-23 00-13-33](https://user-images.githubusercontent.com/31202659/213945590-80da1f25-1369-46f4-b1b9-8fc54c85df40.png)|![Screenshot from 2023-01-23 00-13-42](https://user-images.githubusercontent.com/31202659/213945598-52a5614a-d52e-4cf1-ba3b-b6c1ce36ef1c.png)
noise: 0.1 std<br>rot: 20&deg;<br>x-trans: 2m|![Screenshot from 2023-01-23 00-16-06](https://user-images.githubusercontent.com/31202659/213945723-7c3690d5-10b2-4713-a47b-d08c5e6ef470.png)|![Screenshot from 2023-01-23 00-16-20](https://user-images.githubusercontent.com/31202659/213945727-47efd828-db34-49bf-9761-3980fe183bcd.png)|![Screenshot from 2023-01-23 00-16-28](https://user-images.githubusercontent.com/31202659/213945730-d22e64be-17be-42b7-bb53-c1604f913444.png)

TO BE UPDATED
Algorithm | Parameters | # Iterations | Time Taken (s) | Rotational Error (Deg) | Translational Error | MSE
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
ICP|noise: 0<br>rot: 10&deg;<br>x-trans: 50m|22|0.293366|1.65682|7.59378|7.20658
TrICP|noise: 0<br>rot: 10&deg;<br>x-trans: 50m|30|0.46121|1.27165|7.24762|5.8861e-09
ICP|noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|26|0.35903|1.67561|13.7599|8.454
TrICP|noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|51|0.733307|1.34639|16.4488|0.964213
ICP|noise: 0<br>rot: 20&deg;<br>x-trans: 0m|19|0.251586|1.55872|1.41798|7.20665
TrICP|noise: 0<br>rot: 20&deg;<br>x-trans: 0m|30|0.419888|0.181308|1.02054|1.05461e-08

## Horse
Parameters | Ground Truth | ICP | TrICP
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
noise: 0<br>rot: 10&deg;<br>x-trans: 10m|![Screenshot from 2023-01-23 00-24-38](https://user-images.githubusercontent.com/31202659/213946152-f619a94c-5e7e-4cd9-8895-c3e8ddfe3294.png)|![Screenshot from 2023-01-23 00-25-11](https://user-images.githubusercontent.com/31202659/213946154-b4926089-1d1c-4cae-82fa-0d25eb6bb05a.png)|![Screenshot from 2023-01-23 00-25-18](https://user-images.githubusercontent.com/31202659/213946158-2a1ef989-29e2-48f7-b2e8-ff5bbcc8c567.png)
noise: 0<br>rot: 20&deg;<br>x-trans: 50m|![Screenshot from 2023-01-23 00-27-35](https://user-images.githubusercontent.com/31202659/213946310-432f0f28-eb22-461b-a573-d43d44d18c93.png)|![Screenshot from 2023-01-23 00-28-26](https://user-images.githubusercontent.com/31202659/213946313-90049c52-81e6-4143-9d99-f5d5d42bbbc0.png)|![Screenshot from 2023-01-23 00-28-53](https://user-images.githubusercontent.com/31202659/213946318-d58215d9-efa4-4eb1-9c49-e6459d10e8d0.png)
noise: 1.0 std<br>rot: 20&deg;<br>x-trans: 10m|![Screenshot from 2023-01-23 00-31-01](https://user-images.githubusercontent.com/31202659/213946390-e53079b3-721f-48ab-91a0-70bd372b1268.png)|![Screenshot from 2023-01-23 00-31-07](https://user-images.githubusercontent.com/31202659/213946393-97b50bcf-bcd9-4a81-bf3a-8f8486b416de.png)|![Screenshot from 2023-01-23 00-31-11](https://user-images.githubusercontent.com/31202659/213946399-21afe6aa-4d10-43b0-ba27-7abbf3d2b159.png)

TO BE UPDATED
Algorithm | Parameters | # Iterations | Time Taken (s) | Rotational Error (Deg) | Translational Error | MSE
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
ICP|noise: 0<br>rot: 10&deg;<br>x-trans: 50m|22|0.293366|1.65682|7.59378|7.20658
TrICP|noise: 0<br>rot: 10&deg;<br>x-trans: 50m|30|0.46121|1.27165|7.24762|5.8861e-09
ICP|noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|26|0.35903|1.67561|13.7599|8.454
TrICP|noise: 1.0 std<br>rot: 10&deg;<br>x-trans: 100m|51|0.733307|1.34639|16.4488|0.964213
ICP|noise: 0<br>rot: 20&deg;<br>x-trans: 0m|19|0.251586|1.55872|1.41798|7.20665
TrICP|noise: 0<br>rot: 20&deg;<br>x-trans: 0m|30|0.419888|0.181308|1.02054|1.05461e-08

## Extra: 3D Reconstruction
The data is taken from the warehouse objects given here: http://rgbd-dataset.cs.washington.edu/dataset/rgbd-scenes-v2/
```
./Reconstruction PC1 PC2 MAX_I noise_std rot_angle_deg trans_dist
```

```
./Reconstruction ../office_chair/office_chair_1_1.ply ../office_chair/office_chair_1_2.ply 500 0 0 0
```

#### Only using two point clouds
![Screenshot from 2023-01-23 00-39-15](https://user-images.githubusercontent.com/31202659/213946779-f7ba10a0-6c97-4d24-8bf1-fcbf9a7cac93.png)

#### Reconstruction from 8 point clouds
![Screenshot from 2023-01-23 00-38-23](https://user-images.githubusercontent.com/31202659/213946791-1f8dc654-cf6e-421e-93ba-063f68bb16cb.png)

The reconstruction is done with ICP
