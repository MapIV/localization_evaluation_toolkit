# localization_evaluation_toolkit -Ver.5.0 (Update 1/20)
You can evaluate your localization result by comparing it to a reliable pose trajectory. The start time, end time, and period can be different for both data. The evaluation is automatically aligned with the one with the smaller number of data.

## Releases
|Version|Release Data| Python version | Note |
|  ---  |  ---  | --- | --- |
| 6.0 | soon | python 3.7 or higher | Support evaluation with ros2bag  |
| 5.0 | January 19, 2023 | python 3.7 or higher | Support multiple types of trajectories (csv evaluation only) |
| 3.2 | September 29, 2022 | python 3.6 or higher | 2 pair evaluation |

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
    - Timestamp
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
- To use 'Display ellipse' tool, refer to **E** in **Sub-evaluation and adjustment**. Prepare ros1 bag file including geometry_msgs/PoseWithCovarianceStamped topic.

**B. Evaluate with ros2 bag files → Use config/read_ros2bag.yaml**  
- Specify topic names in 'Reference' and 'Result'.

**Display, Save column**  
- 'Display' column is the setting of output graphs. In 'Save' column, you can choose whether to save the graphs.

## How to run
**A. Evaluate with csv files**
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py [evaluation.yaml path]
```

**B. Evaluate with ros2 bag files**
```
$ cd localization_evaluation_toolkit/scripts
$ python3 ros2bag_main.py [reference_bag_path] [result_bag_path] [read_ros2bag.yaml path] [output_folder_path]
```

output_folder_path should be without "/"

## Sub-evaluation and adjustment
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

**E. Create csv file with covariance column from rosbag file**  
In detail, refer to [qiita article](https://qiita.com/koki2022/items/148d56e0f8eee45a0a62)
```
$ cd localization_evaluation_toolkit/sub_scripts
$ python covariance_to_csv.py [input_bag_path] [output_folder_path]
```

## What graphs are output?
Sample output graphs are shown below. You can test this evaluation script easily with the following command. All graphs are output in full HD.
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py ../sample_data/config/sasashima_evaluation.yaml
```

1. 2D Trajectory

![2d_traj](/sample_data/output_sample/2d_trajectory.png)

When you zoom in on the graph, you can see the correspondence.
![2D_traj_zoom](/sample_data/output_sample/2d_trajectory_zoom.png)

2. 2D Error

![2d_error](/sample_data/output_sample/2d_error.png)

3. Height Error

![height_error](/sample_data/output_sample/height_error.png)

4. 3D Error

![3d_error](/sample_data/output_sample/3d_error.png)

5. Longitudinal Error

![longitudinal_error](/sample_data/output_sample/longitudinal_error.png)

6. Lateral Error

![lateral_error](/sample_data/output_sample/lateral_error.png)

7. Roll Pitch Yaw

You can choose between radian display and degree display.
![rpy](/sample_data/output_sample/rpy.png)

8. Roll Pitch Yaw Error

You can choose between radian display and degree display.
![rpy_error](/sample_data/output_sample/rpy_error.png)
