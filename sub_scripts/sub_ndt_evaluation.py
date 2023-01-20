import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# sys.path.append("../scripts")
from scripts.util import read_ros2bag


class BagParam:
    def __init__(self):

        # Bag
        self.topic = ""
        self.bag_id = "sqlite3"
        self.bag_format = "cdr"

        # DataFrame
        self.df = pd.DataFrame()
        self.df_temp = pd.DataFrame()


if __name__ == "__main__":

    argv = sys.argv
    bag_file = argv[1]
    output_dir = argv[2]

    pose_ = BagParam()
    pose_.topic = "/localization/pose_estimator/pose_with_covariance"
    read_ros2bag.read_pose(bag_file, pose_)
    pose_empty = pose_.df_temp.empty

    nvtl_ = BagParam()
    nvtl_.topic = "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
    read_ros2bag.read_unique(bag_file, nvtl_)
    nvtl_empty = nvtl_.df_temp.empty

    tp_ = BagParam()
    tp_.topic = "/localization/pose_estimator/transform_probability"
    read_ros2bag.read_unique(bag_file, tp_)
    tp_empty = tp_.df_temp.empty

    exe_time_ = BagParam()
    exe_time_.topic = "/localization/pose_estimator/exe_time_ms"
    read_ros2bag.read_unique(bag_file, exe_time_)
    exe_time_empty = exe_time_.df_temp.empty

    itr_ = BagParam()
    itr_.topic = "/localization/pose_estimator/iteration_num"
    read_ros2bag.read_unique(bag_file, itr_)
    itr_empty = itr_.df_temp.empty

    # Plot
    if pose_empty == False and nvtl_empty == False:
        for i in range(0, len(pose_.df_temp)):
            search_sync = abs(nvtl_.df_temp.iloc[:, 0] - pose_.df_temp.iloc[i, 0])
            sync_id = search_sync.idxmin()
            nvtl_.df = nvtl_.df.append(nvtl_.df_temp.iloc[sync_id, :], ignore_index=True)
        nvtl_.df.reset_index(inplace=True, drop=True)

        fig_trj_nvtl = plt.figure("Trajectory and NVTL", figsize=(16, 9), dpi=120)
        ax_trj_nvtl = fig_trj_nvtl.add_subplot(111)
        ax_trj_nvtl.set_title("Trajectory and NVTL")
        scatter_nvtl = ax_trj_nvtl.scatter(
            pose_.df_temp["x"], pose_.df_temp["y"], c=nvtl_.df["data"], vmin=0.0, vmax=5, cmap="jet"
        )
        plt.colorbar(scatter_nvtl, label="NVTL")
        ax_trj_nvtl.set_xlabel("x[m]")
        ax_trj_nvtl.set_ylabel("y[m]")
        ax_trj_nvtl.set_aspect("equal")
        ax_trj_nvtl.grid()
        fig_trj_nvtl.savefig(output_dir + "/trj_nvtl.png")

    if pose_empty == False and tp_empty == False:
        for i in range(0, len(pose_.df_temp)):
            search_sync = abs(tp_.df_temp.iloc[:, 0] - pose_.df_temp.iloc[i, 0])
            sync_id = search_sync.idxmin()
            tp_.df = tp_.df.append(tp_.df_temp.iloc[sync_id, :], ignore_index=True)
        tp_.df.reset_index(inplace=True, drop=True)

        fig_trj_tp = plt.figure("Trajectory and TP", figsize=(16, 9), dpi=120)
        ax_trj_tp = fig_trj_tp.add_subplot(111)
        ax_trj_tp.set_title("Trajectory and TP")
        scatter_tp = ax_trj_tp.scatter(
            pose_.df_temp["x"], pose_.df_temp["y"], c=tp_.df["data"], vmin=0.0, vmax=7.0, cmap="jet"
        )
        plt.colorbar(scatter_tp, label="TP")
        ax_trj_tp.set_xlabel("x[m]")
        ax_trj_tp.set_ylabel("y[m]")
        ax_trj_tp.set_aspect("equal")
        ax_trj_tp.grid()
        fig_trj_tp.savefig(output_dir + "/trj_tp.png")

    if nvtl_empty == False:
        fig_nvtl = plt.figure("NVTL", figsize=(16, 9), dpi=120)
        ax_nvtl = fig_nvtl.add_subplot(111)
        ax_nvtl.set_title("NVTL")
        ax_nvtl.plot(
            nvtl_.df_temp["time"] - nvtl_.df_temp.at[0, "time"], nvtl_.df_temp["data"], marker="o", c="b", markersize=2
        )
        ax_nvtl.set_xlabel("time[s]")
        ax_nvtl.set_ylabel("NVTL")
        ax_nvtl.set_ylim(0, 5)
        ax_nvtl.grid()
        fig_nvtl.savefig(output_dir + "/nvtl.png")

    if tp_empty == False:
        fig_tp = plt.figure("TP", figsize=(16, 9), dpi=120)
        ax_tp = fig_tp.add_subplot(111)
        ax_tp.set_title("TP")
        ax_tp.plot(
            tp_.df_temp["time"] - tp_.df_temp.at[0, "time"], tp_.df_temp["data"], marker="o", c="b", markersize=2
        )
        ax_tp.set_xlabel("time[s]")
        ax_tp.set_ylabel("TP")
        ax_tp.set_ylim(0, 7)
        ax_tp.grid()
        fig_tp.savefig(output_dir + "/tp.png")

    if exe_time_empty == False:
        fig_exe_time = plt.figure("Execution Time", figsize=(16, 9), dpi=120)
        ax_exe_time = fig_exe_time.add_subplot(111)
        ax_exe_time.set_title("Execution Time")
        ax_exe_time.plot(
            exe_time_.df_temp["time"] - exe_time_.df_temp.at[0, "time"],
            exe_time_.df_temp["data"],
            marker="o",
            c="b",
            markersize=2,
        )
        ax_exe_time.set_xlabel("time[s]")
        ax_exe_time.set_ylabel("Execution Time")
        ax_exe_time.set_ylim(0, 150)
        ax_exe_time.grid()
        fig_exe_time.savefig(output_dir + "/exe_time.png")

    if itr_empty == False:
        fig_itr = plt.figure("Iteration", figsize=(16, 9), dpi=120)
        ax_itr = fig_itr.add_subplot(111)
        ax_itr.set_title("Iteration")
        ax_itr.plot(
            itr_.df_temp["time"] - itr_.df_temp.at[0, "time"], itr_.df_temp["data"], marker="o", c="b", markersize=2
        )
        ax_itr.set_xlabel("time[s]")
        ax_itr.set_ylabel("Iteration")
        ax_itr.grid()
        fig_itr.savefig(output_dir + "/itr.png")

    plt.show()
