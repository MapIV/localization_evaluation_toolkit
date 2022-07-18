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

    pose_data_dict = {
        "time": [],
        "x": [],
        "y": [],
        "z": [],
        "roll": [],
        "pitch": [],
        "yaw": []
    }
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        q_temp = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        e_temp = R.from_quat([q_temp[0], q_temp[1], q_temp[2], q_temp[3]])

        pose_data_dict["time"].append((msg.header.stamp.sec)+(msg.header.stamp.nanosec) / 10**9 + param.tf_time)
        pose_data_dict["x"].append(msg.pose.pose.position.x + param.tf_x)
        pose_data_dict["y"].append(msg.pose.pose.position.y + param.tf_y)
        pose_data_dict["z"].append(msg.pose.pose.position.z + param.tf_z)
        pose_data_dict["roll"].append((e_temp.as_euler("ZYX", degrees=False)[2] + param.tf_roll) * param.inv_roll)
        pose_data_dict["pitch"].append((e_temp.as_euler("ZYX", degrees=False)[1] + param.tf_pitch) * param.inv_pitch)
        pose_data_dict["yaw"].append((e_temp.as_euler("ZYX", degrees=False)[0] + param.tf_yaw) * param.inv_yaw)
    param.df_temp = pd.DataFrame.from_dict(pose_data_dict)


def read_pose(bag_file, param):

    bag_path = str(bag_file)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=param.bag_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format=param.bag_format, output_serialization_format=param.bag_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=[param.topic])
    reader.set_filter(storage_filter)

    pose_data_dict = {
        "time": [],
        "x": [],
        "y": [],
        "z": [],
        "roll": [],
        "pitch": [],
        "yaw": []
    }
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        q_temp = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        e_temp = R.from_quat([q_temp[0], q_temp[1], q_temp[2], q_temp[3]]) 
        pose_data_dict["time"].append((msg.header.stamp.sec)+(msg.header.stamp.nanosec) / 10**9)
        pose_data_dict["x"].append(msg.pose.pose.position.x)
        pose_data_dict["y"].append(msg.pose.pose.position.y)
        pose_data_dict["z"].append(msg.pose.pose.position.z)
        pose_data_dict["roll"].append(e_temp.as_euler("ZYX", degrees=False)[2])
        pose_data_dict["pitch"].append(e_temp.as_euler("ZYX", degrees=False)[1])
        pose_data_dict["yaw"].append(e_temp.as_euler("ZYX", degrees=False)[0])
    param.df_temp = pd.DataFrame.from_dict(pose_data_dict)
    

def read_unique(bag_file, param):
    bag_path = str(bag_file)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=param.bag_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format=param.bag_format, output_serialization_format=param.bag_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=[param.topic])
    reader.set_filter(storage_filter)

    data_dict = {
        "time": [],
        "data": []
    }
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        data_dict["time"].append(msg.stamp.sec + msg.stamp.nanosec / 10**9)
        data_dict["data"].append(msg.data)
    param.df_temp = pd.DataFrame.from_dict(data_dict)