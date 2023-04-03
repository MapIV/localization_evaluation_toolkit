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
from scripts.deprecated import adjust, read_ros2bag, yaml_param


class BagParam:
    def __init__(self):

        # Bag
        self.topic = ""
        self.bag_id = "sqlite3"
        self.bag_format = "cdr"

        # DataFrame
        self.df = pd.DataFrame()
        self.df_temp = pd.DataFrame()


def set_param(pose_param, op_param):
    pose_param.topic = ""
    pose_param.bag_id = "sqlite3"
    pose_param.bag_format = "cdr"
    pose_param.tf_time = 0.0
    pose_param.tf_x = 0.0
    pose_param.tf_y = 0.0
    pose_param.tf_z = 0.0
    pose_param.tf_roll = 0.0
    pose_param.tf_pitch = 0.0
    pose_param.tf_yaw = 0.0
    pose_param.inv_roll = 1.0
    pose_param.inv_pitch = 1.0
    pose_param.inv_yaw = 1.0

    op_param.display_ellipse = True


if __name__ == "__main__":

    argv = sys.argv
    bag_file = argv[1]
    output_dir = argv[2]

    pose_param = yaml_param.YamlParam()
    op_param = yaml_param.OpParam()
    set_param(pose_param, op_param)
    pose_param.topic = "/localization/pose_estimator/pose_with_covariance"
    read_ros2bag.read_ros2bag(bag_file, pose_param, op_param)
    pose_param.df = pose_param.df_temp.copy()
    pose_empty = pose_param.df.empty

    nvtl = BagParam()
    nvtl.topic = "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
    read_ros2bag.read_unique(bag_file, nvtl)
    nvtl_empty = nvtl.df_temp.empty

    tp = BagParam()
    tp.topic = "/localization/pose_estimator/transform_probability"
    read_ros2bag.read_unique(bag_file, tp)
    tp_empty = tp.df_temp.empty

    exe_time = BagParam()
    exe_time.topic = "/localization/pose_estimator/exe_time_ms"
    read_ros2bag.read_unique(bag_file, exe_time)
    exe_time_empty = exe_time.df_temp.empty

    itr = BagParam()
    itr.topic = "/localization/pose_estimator/iteration_num"
    read_ros2bag.read_unique(bag_file, itr)
    itr_empty = itr.df_temp.empty

    # Plot
    if pose_empty == False and nvtl_empty == False:
        for i in range(0, len(pose_param.df)):
            search_sync = abs(nvtl.df_temp.iloc[:, 0] - pose_param.df.iloc[i, 0])
            sync_id = search_sync.idxmin()
            nvtl.df = pd.concat([nvtl.df, nvtl.df_temp.iloc[sync_id, :]], ignore_index=True, axis=1)
        nvtl.df.reset_index(inplace=True, drop=True)

        fig_trj_nvtl = plt.figure("Trajectory and NVTL", figsize=(16, 9), dpi=120)
        ax_trj_nvtl = fig_trj_nvtl.add_subplot(111)
        ax_trj_nvtl.set_title("Trajectory and NVTL")
        scatter_nvtl = ax_trj_nvtl.scatter(
            pose_param.df["x"], pose_param.df["y"], c=nvtl.df["data"], vmin=0.0, vmax=5, cmap="jet"
        )
        plt.colorbar(scatter_nvtl, label="NVTL")
        ax_trj_nvtl.set_xlabel("x[m]")
        ax_trj_nvtl.set_ylabel("y[m]")
        ax_trj_nvtl.set_aspect("equal")
        ax_trj_nvtl.grid()
        fig_trj_nvtl.savefig(output_dir + "/trj_nvtl.png")

    if pose_empty == False and tp_empty == False:
        for i in range(0, len(pose_param.df)):
            search_sync = abs(tp.df.iloc[:, 0] - pose_param.df.iloc[i, 0])
            sync_id = search_sync.idxmin()
            tp.df = pd.concat([tp.df, tp.df_temp.iloc[sync_id, :]], ignore_index=True, axis=1)
        tp.df.reset_index(inplace=True, drop=True)

        fig_trj_tp = plt.figure("Trajectory and TP", figsize=(16, 9), dpi=120)
        ax_trj_tp = fig_trj_tp.add_subplot(111)
        ax_trj_tp.set_title("Trajectory and TP")
        scatter_tp = ax_trj_tp.scatter(
            pose_param.df["x"], pose_param.df["y"], c=tp.df["data"], vmin=0.0, vmax=7.0, cmap="jet"
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
        start_time = nvtl.df_temp.at[0, "time"]
        ax_nvtl.plot(nvtl.df_temp["time"] - start_time, nvtl.df_temp["data"], marker="o", c="b", markersize=2)
        ax_nvtl.set_xlabel("time[s]")
        ax_nvtl.set_ylabel("NVTL")
        ax_nvtl.set_ylim(0, 5)
        ax_nvtl.grid()
        fig_nvtl.savefig(output_dir + "/nvtl.png")

    if tp_empty == False:
        fig_tp = plt.figure("TP", figsize=(16, 9), dpi=120)
        ax_tp = fig_tp.add_subplot(111)
        ax_tp.set_title("TP")
        start_time = tp.df_temp.at[0, "time"]
        ax_tp.plot(tp.df_temp["time"] - start_time, tp.df_temp["data"], marker="o", c="b", markersize=2)
        ax_tp.set_xlabel("time[s]")
        ax_tp.set_ylabel("TP")
        ax_tp.set_ylim(0, 7)
        ax_tp.grid()
        fig_tp.savefig(output_dir + "/tp.png")

    if exe_time_empty == False:
        fig_exe_time = plt.figure("Execution Time", figsize=(16, 9), dpi=120)
        ax_exe_time = fig_exe_time.add_subplot(111)
        ax_exe_time.set_title("Execution Time")
        start_time = exe_time.df_temp.at[0, "time"]
        ax_exe_time.plot(
            exe_time.df_temp["time"] - start_time,
            exe_time.df_temp["data"],
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
        start_time = itr.df_temp.at[0, "time"]
        ax_itr.plot(itr.df_temp["time"] - start_time, itr.df_temp["data"], marker="o", c="b", markersize=2)
        ax_itr.set_xlabel("time[s]")
        ax_itr.set_ylabel("Iteration")
        ax_itr.grid()
        fig_itr.savefig(output_dir + "/itr.png")

    if pose_empty == False:
        adjust.calc_ellipse(pose_param)
        fig_ellipse = plt.figure("Error Ellipse", figsize=(16, 9), dpi=120)
        ax_ellipse = fig_ellipse.add_subplot(111)
        ax_ellipse.set_title("Error Ellipse")
        start_time = pose_param.df.at[0, "time"]
        print(pose_param.df["time"])
        print("long")
        print(pose_param.df["ellipse_long"])
        ax_ellipse.plot(
            pose_param.df["time"] - start_time,
            pose_param.df["ellipse_long"],
            marker="o",
            markersize=2,
            label="ellipse long",
        )

        ax_ellipse.plot(
            pose_param.df["time"] - start_time,
            pose_param.df["ellipse_short"],
            marker="o",
            markersize=2,
            label="ellipse short",
        )
        ax_ellipse.set_xlabel("time[s]")
        ax_ellipse.set_ylabel("error ellipse size")
        ax_ellipse.grid()
        fig_exe_time.savefig(output_dir + "/exe_time.png")

    plt.show()
