import pandas as pd
import numpy as np
import sys
import rosbag2_py
from scipy.spatial.transform import Rotation as R

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_ros2bag(bag_file, param):

    bag_path = str(bag_file)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=param.bag_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format=param.bag_format, output_serialization_format=param.bag_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=[param.topic])
    reader.set_filter(storage_filter)

    i = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        param.df_temp.at[i, "time"] = (msg.header.stamp.sec)+(msg.header.stamp.nanosec) / 10**9 + param.tf_time
        param.df_temp.at[i, "x"] = msg.pose.pose.position.x + param.tf_x
        param.df_temp.at[i, "y"] = msg.pose.pose.position.y + param.tf_y
        param.df_temp.at[i, "z"] = msg.pose.pose.position.z + param.tf_z
        q_temp = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        e_temp = R.from_quat([q_temp[0], q_temp[1], q_temp[2], q_temp[3]])
        param.df_temp.at[i, "roll"] = e_temp.as_euler("ZYX", degrees=False)[2] + param.tf_roll
        param.df_temp.at[i, "pitch"] = e_temp.as_euler("ZYX", degrees=False)[1] + param.tf_pitch
        param.df_temp.at[i, "yaw"] = e_temp.as_euler("ZYX", degrees=False)[0] + param.tf_yaw
        i += 1

def read_unique(bag_file, param):
    reader_nvtl = rosbag2_py.SequentialReader()
    reader_nvtl.open(storage_options, converter_options)
    topic_types_nvtl = reader_nvtl.get_all_topics_and_types()

    type_map_nvtl = {topic_types_nvtl[i].name: topic_types_nvtl[i].type for i in range(len(topic_types_nvtl))}

    nvtl_time = []
    nvtl = []

    storage_filter_nvtl = rosbag2_py.StorageFilter(topics=['/localization/pose_estimator/nearest_voxel_transformation_likelihood'])
    reader_nvtl.set_filter(storage_filter_nvtl)

    msg_counter = 0
    while reader_nvtl.has_next():
        (topic, data, t) = reader_nvtl.read_next()
        msg_type = get_message(type_map_nvtl[topic])
        msg = deserialize_message(data, msg_type)
        nvtl_time.append((msg.stamp.sec)+(msg.stamp.nanosec)/ 10**9)
        nvtl.append(msg.data)
        # print(msg.data)
        msg_counter += 1

    df_nvtl_t = pd.DataFrame(nvtl_time)
    df_nvtl = pd.DataFrame(nvtl)
