# localization_evaluation_toolkit -Ver.3.2
You can evaluate your a localization result by comparing it to a reliable pose trajectory. The start time, end time, and period can be different for both data. The evaluation is automatically aligned with the one with the smaller number of data.

## Install
```
$ git clone https://github.com/MapIV/localization_evaluation_toolkit.git
```

## Necessary files
You should prepare the two types of csv files or bag files. The start time, end time, and period can be different for both data.  
|Item|Description|
|  ---  |  ---  |
| Reference | Reliable pose trajectory |
| Result | Localization result trajectory you'd like to evaluate |

**A. Evaluate with csv files**  
- csv files require at least the following elements.
    - Time stamp
    - Position (x,y,z)
    - Rotation (Quaternion or Euler[degree or radian])

**B. Evaluate with ros2 bag files**  
- ros2 bag files require one of the following types of topics.
    - Type: geometry_msgs/msg/PoseWithCovarianceStamped
    - Type: nav_msgs/msg/Odometry

## How to specify the value of yaml file
All yaml files are in the config/ directory.

**A. Evaluate with csv files → Use config/evaluation.yaml**  
- In 'Reference' and 'Result', specify the number of csv columns corresponding to the element. 
- Depending on your csv, select the time and rotation format as True or False.

**B. Evaluate with ros2 bag files → Use config/read_ros2bag.yaml**  
- Specify topic names in 'Reference' and 'Result'.

**Display, Save column**  
- 'Display' column is setting of output graphs. In 'Save' column, you can choose whether to save the graphs.

## How to run
**A. Evaluate with csv files**
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py [reference_csv_path] [result_csv_path] [evaluation.yaml path] [output_folder_path]
```

**B. Evaluate with ros2 bag files**
```
$ cd localization_evaluation_toolkit/scripts
$ python3 ros2bag_main.py [reference_bag_path] [result_bag_path] [read_ros2bag.yaml path] [output_folder_path]
```

output_folder_path shoule be without "/"

## Sub evaluation and adjustment
**C. Evaluate TP, NVTL, execution time and iteration with ros2 bag files**

Please input a rosbag containing the following (at least one) topic.
```
/localization/pose_estimator/pose_with_covariance
/localization/pose_estimator/nearest_voxel_transformation_likelihood
/localization/pose_estimator/transform_probability
/localization/pose_estimator/exe_time_ms
/localization/pose_estimator/iteration_num
```

```
$ cd localization_evaluation_toolkit/sub_scripts
$ source ~/xxxxxx/install/setup.bash
$ python3 sub_ndt_evaluation.py [bag_path] [output_folder_path]
```

**D. Adjust time stamp with csv files**
```
$ cd localization_evaluation_toolkit/sub_scripts
$ python3 adjust_time_stamp.py [target_csv_path] [offset_csv_path] [ajust_time_stamp.yaml path] [output_folder_path]
```

## What graphs are outputed?
Sample output graphs are shown below. You can test this evaluation script easily with the following command. All graphs are outputed in full HD.
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py ~/localization_evaluation_toolkit/sample_data/reference.csv ~/localization_evaluation_toolkit/sample_data/result_trj_ndt.csv ~/localization_evaluation_toolkit/sample_data/config/sasashima_evaluation.yaml ~/localization_evaluation_toolkit/sample_data/output_test
```

1. 2D Trajectory

![2d_trj](https://user-images.githubusercontent.com/81670028/177121453-eb6c2c35-15f8-4769-a3ee-f80fc91526ec.png)

When you zoom in on the graph, you can see the correspondence.
![2D_Trajectory_zoom](https://user-images.githubusercontent.com/81670028/177121881-c157dbf5-6829-471a-b923-352ac31c14e2.png)

2. 2D Error

![2d_error](https://user-images.githubusercontent.com/81670028/177121927-f8519619-e300-46bb-adf3-62741519a2fb.png)

3. Height Error

![height_error](https://user-images.githubusercontent.com/81670028/177121971-71db77a4-cf89-4550-a563-8739f893a6a7.png)

4. 3D Error

![3d_error](https://user-images.githubusercontent.com/81670028/177122017-a0e06e5c-3fa6-41f1-a6a0-cf06debd4074.png)

5. Longitudinal Error

![longitudinal_error](https://user-images.githubusercontent.com/81670028/177122086-28a5d4db-3bd2-4d19-9a9c-167e9974daf8.png)

6. Lateral Error

![lateral_error](https://user-images.githubusercontent.com/81670028/177122142-a5ec259d-d1eb-4c02-b11a-08ffdecb6ae0.png)

7. Roll Pitch Yaw

You can choose between radian display and degree display.
![rpy](https://user-images.githubusercontent.com/81670028/177122197-a3686219-a840-4844-bdc3-6661f8d3c55f.png)

8. Roll Pitch Yaw Error

You can choose between radian display and degree display.
![rpy_error](https://user-images.githubusercontent.com/81670028/177122246-c4c30803-9e25-45d0-aa16-d7d87d5091f2.png)
