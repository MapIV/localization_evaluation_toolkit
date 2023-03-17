# localization_evaluation_toolkit -Ver.6.1 (Update 2023/03/14)
You can evaluate your localization result by comparing it to a reliable pose trajectory.
The start time, end time, and period can be different for each data.
The evaluation is automatically aligned with the one with the smaller number of data.

## Releases
| Version | Release Date       | Python version       | Note                                                         |
| ------- | ------------------ | -------------------- | ------------------------------------------------------------ |
| 6.0     | Feburary 3, 2023   | python 3.7 or higher | Support multi-file evaluation (mixtures of csv and ros2 bag) |
| 5.0     | January 20, 2023   | python 3.7 or higher | Support multi-file evaluation (CSV only)                     |
| 3.2     | September 29, 2022 | python 3.6 or higher | 1:1 evaluation                                               |

## Installation

```sh
$ git clone https://github.com/MapIV/localization_evaluation_toolkit.git
```

## Preparation

Evaluatable data types are limited to the format of CSV or ros2 bag, and the start time, end time, and period can be different for each data.
Specifically, a valid **CSV file** requires at least the following entries.

- Timestamp
- Position (x,y,z)
- Rotation (Quaternion or Euler[degree or radian])

And a valid ros2 bag file should have one of the following types of topics.

- `geometry_msgs/msg/PoseWithCovarianceStamped`
- `nav_msgs/msg/Odometry`

## Execution

A standard configuration file for evaluation consists of the following parts.

- Reference
- Result1, Result2, ...
- Optional settings

It is required that the reference block should have the name of ***Reference*** and the names of result blocks should begin with ***Result***.
When specifying the paths, it is recommended to use the absolute path.
There is a sample YAML file at `config/evaluation.yaml`.
To create your own configuration file, just use the following templates for CSV or ros2 bag,

<details>
<summary>Template of CSV (click to expand)</summary>

```yaml
Reference/Result:
  ## Auxiliary info
  label: any
  type: 0 # [0]: csv, [1]: ros2bag
  path: /path/to/csv

  ## Time
  separate_time_stamp: false # [true]:Set secs_stamp_column and nsecs_stamp_column / [false]:Set stamp_column
  #--------true--------#
  secs_stamp_column: 2
  nsecs_stamp_column: 3
  #--------false-------#
  stamp_column: 0

  ## Position
  x_column: 1
  y_column: 2
  z_column: 3

  ## Rotation
  use_quaternion: false # [true]:Set Quaternion / [false]:Set Euler
  #--------true--------#
  # Quaternion
  ori_x_column: 8
  ori_y_column: 9
  ori_z_column: 10
  ori_w_column: 11
  #--------false-------#
  # Euler
  use_radian: true # [true]:radian / [false]:degree
  roll_column: 4
  pitch_column: 5
  yaw_column: 6

  ## TF
  tf_time: 0   # [s]
  tf_x: 0      # [m]
  tf_y: 0      # [m]
  tf_z: 0      # [m]
  tf_roll: 0   # [rad]
  tf_pitch: 0  # [rad]
  tf_yaw: 0    # [rad]
  inv_roll: 1  # 1 or -1
  inv_pitch: 1 # 1 or -1
  inv_yaw: 1   # 1 or -1

  # Display ellipse (put 2D covariance in result file)
  display_ellipse: false
  covariance_xx_column: 10
  covariance_xy_column: 11
  covariance_yx_column: 12
  covariance_yy_column: 13
```

</details>

<details>
<summary>Template of ros2 bag (click to expand)</summary>

```yaml
Reference/Result:
  ## Auxiliary info
  label: any
  type: 1 # [0]: csv, [1]: ros2bag
  path: /path/to/ros2bag

  ## Rosbag info
  topic_name: /localization/pose_estimator/pose_with_covariance
  storage_id: sqlite3
  serialization_format: cdr

  ## TF
  tf_time: 0   # [s]
  tf_x: 0      # [m]
  tf_y: 0      # [m]
  tf_z: 0      # [m]
  tf_roll: 0   # [rad]
  tf_pitch: 0  # [rad]
  tf_yaw: 0    # [rad]
  inv_roll: 1  # 1 or -1
  inv_pitch: 1 # 1 or -1
  inv_yaw: 1   # 1 or -1

  # Display ellipse (put 2D covariance in result file)
  display_ellipse: false # use PoseWithCovarianceStamped tyep topic in result data
```

</details>

<details>
<summary>Template of optional settings (click to expand)</summary>

```yaml
# Horizontal axis
axis_type: 0   # [0]:time, [1]:distance
degree_type: 0 # [0]:radian, [1]:degree

# Trajectory graph dilution
dilution_step: 10 # at least 1, the larger the sparser for better performance

# Trajectory graph numbering
progress_info: 0 # [0]:off, [1]:number, [2]:time, [3]:ros time, [4]:distance
interval: 0      # progress_info is [2]:second, [3]:second, [4]:meter

# Font
title_font_size: 14 
label_font_size: 10
ticks_font_size: 8

# Save
save_figures: true
save_extension_type: png # without "."
save_dataframe: true
output_directory: /path/to/output/directory

use_lerp: false
```

</details>

and run with

```sh
$ cd localization_evaluation_toolkit
$ python3 scripts/main.py config/evaluation.yaml
```

## Sub-evaluation and adjustment

**A. Evaluate TP, NVTL, execution time and iteration with ros2 bag files**

Please input a rosbag containing the following (at least one) topic.
```
/localization/pose_estimator/pose_with_covariance
/localization/pose_estimator/nearest_voxel_transformation_likelihood
/localization/pose_estimator/transform_probability
/localization/pose_estimator/exe_time_ms
/localization/pose_estimator/iteration_num
```

```sh
$ cd localization_evaluation_toolkit/sub_scripts
$ source ~/xxxxxx/install/setup.bash
$ python3 sub_ndt_evaluation.py [bag_path] [output_folder_path]
```

**B. Adjust time stamp with CSV files**
```sh
$ cd localization_evaluation_toolkit/sub_scripts
$ python3 adjust_time_stamp.py [target_csv_path] [offset_csv_path] [ajust_time_stamp.yaml path] [output_folder_path]
```

**C. Create CSV file with covariance column from rosbag file**  
In detail, refer to [qiita article](https://qiita.com/koki2022/items/148d56e0f8eee45a0a62)
```sh
$ cd localization_evaluation_toolkit/sub_scripts
$ python covariance_to_csv.py [input_bag_path] [output_folder_path]
```

## Output graphs
Sample output graphs are shown below. You can test this evaluation script easily with the following command. All graphs are output in full HD.
```sh
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

You can choose the unit of the vertical axis between radian and degree.
![rpy](/sample_data/output_sample/rpy.png)

8. Roll Pitch Yaw Error

You can choose the unit of the vertical axis between radian and degree.
![rpy_error](/sample_data/output_sample/rpy_error.png)
