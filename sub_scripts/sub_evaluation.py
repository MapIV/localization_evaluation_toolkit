import pandas as pd
import numpy as np
import sys
import rosbag2_py
import matplotlib.pyplot as plt
import math

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

if __name__ == "__main__":

    argv = sys.argv
    bag_file = argv[1]
    # tp_csv = argv[2]
    # nvtl_csv = argv[3]
    # iteration_csv = argv[4]

    bag_path = str(bag_file)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # NDT
    storage_filter = rosbag2_py.StorageFilter(topics=['/localization/pose_estimator/pose_with_covariance'])
    reader.set_filter(storage_filter)

    ndt_pose_time = []
    ndt_pose_x = []
    ndt_pose_y = []

    msg_counter = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        ndt_pose_time.append((msg.header.stamp.sec)+(msg.header.stamp.nanosec) / 10**9)
        ndt_pose_x.append(msg.pose.pose.position.x)
        ndt_pose_y.append(msg.pose.pose.position.y)
        msg_counter += 1

    df_ndt_t = pd.DataFrame(ndt_pose_time)
    df_ndt_x = pd.DataFrame(ndt_pose_x)
    df_ndt_y = pd.DataFrame(ndt_pose_y)

    # NVTL
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

    # Sync time
    sync_nvtl = []
    for i in range(0, len(ndt_pose_x)):
        search_sync_ref_time = abs(df_nvtl_t.iloc[:,0] - df_ndt_t.iloc[i,0])
        sync_ref_id = search_sync_ref_time.idxmin()
        min_ref_time = search_sync_ref_time.min()
        sync_nvtl.append(df_nvtl.iloc[sync_ref_id])
    df_nvtl = pd.DataFrame(sync_nvtl)
    df_nvtl.reset_index(inplace=True, drop=True)

    # Plot
    fig_trj_nvtl = plt.figure("Trajectory and NVTL", figsize=(16, 9), dpi=120)
    ax_trj_nvtl = fig_trj_nvtl.add_subplot(111)
    ax_trj_nvtl.set_title("Trajectory and NVTL")
    scatter_nvtl = ax_trj_nvtl.scatter(df_ndt_x, df_ndt_y, c=df_nvtl, cmap='jet_r')
    plt.colorbar(scatter_nvtl, label="NVTL")
    ax_trj_nvtl.set_aspect("equal")
    ax_trj_nvtl.grid()

    # fig_trj_nvtl = plt.figure("Trajectory and NVTL", figsize=(16, 9), dpi=120)
    # ax_trj_nvtl = fig_trj_nvtl.add_subplot(111)
    # ax_trj_nvtl.set_title("Trajectory and NVTL")
    # scatter_nvtl = ax_trj_nvtl.scatter(convergence_data["current_pose.x"], convergence_data["current_pose.y"], c=convergence_data["nvtl"], cmap=cm.jet, linewidth=0)
    # plt.colorbar(scatter_nvtl, label="NVTL")
    # ax_trj_nvtl.set_aspect("equal")
    # ax_trj_nvtl.grid()

    fig_plot_nvtl = plt.figure("NVTL", figsize=(16, 9), dpi=120)
    ax_plot_nvtl = fig_plot_nvtl.add_subplot(111)
    ax_plot_nvtl.set_title("NVTL")
    ax_plot_nvtl.plot(df_ndt_t - df_ndt_t[0], df_nvtl, marker="o", c="b", markersize=2)
    ax_plot_nvtl.set_xlabel("time[s]")
    ax_plot_nvtl.set_ylabel("NVTL")
    ax_plot_nvtl.grid()

    # fig_tp_nvtl = plt.figure("TP and NVTL", figsize=(16, 9), dpi=120)
    # ax_tp_nvtl = fig_tp_nvtl.add_subplot(111)
    # ax_tp_nvtl.set_title("TP and NVTL")
    # ax_tp_nvtl.plot(df_ndt_t, df_nvtl, marker="o", c="b", markersize=2)
    # # ax_tp_nvtl.plot(???, ???, marker="o", c="r", markersize=2)
    # ax_tp_nvtl.set_xlabel("time[s]")
    # ax_tp_nvtl.set_ylabel("TP, NVTL")
    # ax_tp_nvtl.grid()

    # fig_itr = plt.figure("Iteration", figsize=(16, 9), dpi=120)
    # ax_itr = fig_itr.add_subplot(111)
    # ax_itr.set_title("Iteration")
    # ax_itr.plot(???, ???, marker="o", c="b", markersize=2)
    # ax_itr.set_xlabel("time[s]")
    # ax_itr.set_ylabel("Iteration")
    # ax_itr.grid()

    plt.show()
