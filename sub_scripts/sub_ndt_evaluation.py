import math
import os
import sys

import matplotlib.patches as patches
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


def plot_trajectory_nvtl(pose_param, nvtl, output_dir):
    for i in range(0, len(pose_param.df)):
        search_sync = abs(nvtl.df_temp.iloc[:, 0] - pose_param.df.iloc[i, 0])
        sync_id = search_sync.idxmin()
        # nvtl.df = nvtl.df.append(nvtl.df_temp.iloc[sync_id, :], ignore_index=True)
        nvtl.df = pd.concat([nvtl.df, nvtl.df_temp.iloc[[sync_id]]], ignore_index=True)
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


def plot_trajectory_tp(pose_param, tp, output_dir):
    for i in range(0, len(pose_param.df)):
        search_sync = abs(tp.df_temp.iloc[:, 0] - pose_param.df.iloc[i, 0])
        sync_id = search_sync.idxmin()
        # tp.df = tp.df.append(tp.df_temp.iloc[sync_id, :], ignore_index=True)
        tp.df = pd.concat([tp.df, tp.df_temp.iloc[[sync_id]]], ignore_index=True)
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


def plot_nvtl(nvtl, output_dir):
    fig_nvtl = plt.figure("NVTL", figsize=(16, 9), dpi=120)
    ax_nvtl = fig_nvtl.add_subplot(111)
    ax_nvtl.set_title("NVTL")
    ax_nvtl.plot(nvtl.df_temp["time"] - nvtl.df_temp.at[0, "time"], nvtl.df_temp["data"], marker="o", markersize=2)
    ax_nvtl.set_xlabel("time[s]")
    ax_nvtl.set_ylabel("NVTL")
    ax_nvtl.set_ylim(0, 5)
    ax_nvtl.grid()
    fig_nvtl.savefig(output_dir + "/nvtl.png")


def plot_tp(tp, output_dir):
    fig_tp = plt.figure("TP", figsize=(16, 9), dpi=120)
    ax_tp = fig_tp.add_subplot(111)
    ax_tp.set_title("TP")
    ax_tp.plot(tp.df_temp["time"] - tp.df_temp.at[0, "time"], tp.df_temp["data"], marker="o", markersize=2)
    ax_tp.set_xlabel("time[s]")
    ax_tp.set_ylabel("TP")
    ax_tp.set_ylim(0, 7)
    ax_tp.grid()
    fig_tp.savefig(output_dir + "/tp.png")


def plot_exe_time(exe_time, output_dir):
    fig_exe_time = plt.figure("Execution Time", figsize=(16, 9), dpi=120)
    ax_exe_time = fig_exe_time.add_subplot(111)
    ax_exe_time.set_title("Execution Time")
    ax_exe_time.plot(
        exe_time.df_temp["time"] - exe_time.df_temp.at[0, "time"],
        exe_time.df_temp["data"],
        marker="o",
        markersize=2,
    )
    ax_exe_time.set_xlabel("time[s]")
    ax_exe_time.set_ylabel("Execution Time")
    ax_exe_time.set_ylim(0, 150)
    ax_exe_time.grid()
    fig_exe_time.savefig(output_dir + "/exe_time.png")


def plot_itr(itr, output_dir):
    fig_itr = plt.figure("Iteration", figsize=(16, 9), dpi=120)
    ax_itr = fig_itr.add_subplot(111)
    ax_itr.set_title("Iteration")
    ax_itr.plot(itr.df_temp["time"] - itr.df_temp.at[0, "time"], itr.df_temp["data"], marker="o", markersize=2)
    ax_itr.set_xlabel("time[s]")
    ax_itr.set_ylabel("Iteration")
    ax_itr.grid()
    fig_itr.savefig(output_dir + "/itr.png")


def plot_ellipse_ls(pose_param, output_dir):
    # ellipse long
    start_time = pose_param.df.at[0, "time"]
    fig_el = plt.figure("Error Ellipse (long/short)", figsize=(16, 9), dpi=120)
    ax_el_long = fig_el.add_subplot(211)
    ax_el_long.set_title("Ellipse Long Radius")
    ax_el_long.plot(
        pose_param.df["time"] - start_time,
        pose_param.df["ellipse_long"],
        marker="o",
        markersize=2,
    )
    ax_el_long.set_xlabel("time[s]")
    ax_el_long.set_ylabel("size[m]")
    ax_el_long.set_ylim(bottom=-0.01)
    ax_el_long.grid()

    # ellipse short
    ax_el_short = fig_el.add_subplot(212)
    ax_el_short.set_title("Ellipse Short Radius")
    ax_el_short.plot(
        pose_param.df["time"] - start_time,
        pose_param.df["ellipse_short"],
        marker="o",
        markersize=2,
    )
    ax_el_short.set_xlabel("time[s]")
    ax_el_short.set_ylabel("size[m]")
    ax_el_short.set_ylim(bottom=-0.01)
    ax_el_short.grid()

    plt.tight_layout()
    fig_el.savefig(output_dir + "/error_ellipse_ls.png")


def plot_ellipse_ll(pose_param, output_dir):
    # ellipse longitudinal direction
    start_time = pose_param.df.at[0, "time"]
    fig_el2 = plt.figure("Error Ellipse (longitudinal/lateral)", figsize=(16, 9), dpi=120)
    ax_el2_longitudinal = fig_el2.add_subplot(211)
    ax_el2_longitudinal.set_title("Ellipse Longitudinal Direction")
    ax_el2_longitudinal.plot(
        pose_param.df["time"] - start_time,
        pose_param.df["ellipse_longitudinal"],
        marker="o",
        markersize=2,
    )
    ax_el2_longitudinal.set_xlabel("time[s]")
    ax_el2_longitudinal.set_ylabel("size[m]")
    ax_el2_longitudinal.set_ylim(bottom=-0.01)
    ax_el2_longitudinal.grid()

    # ellipse lateral direction
    ax_el2_lateral = fig_el2.add_subplot(212)
    ax_el2_lateral.set_title("Ellipse Lateral Direction")
    ax_el2_lateral.plot(
        pose_param.df["time"] - start_time,
        pose_param.df["ellipse_lateral"],
        marker="o",
        markersize=2,
    )
    ax_el2_lateral.set_xlabel("time[s]")
    ax_el2_lateral.set_ylabel("size[m]")
    ax_el2_lateral.set_ylim(bottom=-0.01)
    ax_el2_lateral.grid()
    plt.tight_layout()
    fig_el2.savefig(output_dir + "/error_ellipse2_ll.png")


def create_ellipse(pose_param, ax):
    for i in range(len(pose_param.df)):
        e = patches.Ellipse(
            xy=(pose_param.df["x"][i], pose_param.df["y"][i]),
            width=pose_param.df["ellipse_long"][i] * 2,
            height=pose_param.df["ellipse_short"][i] * 2,
            angle=math.degrees(pose_param.df["ellipse_yaw"][i]),
            alpha=0.3,
            color="m",
        )
        ax.add_patch(e)


def plot_ellipse_trj(pose_param, output_dir):
    fig_trj_el_long = plt.figure("Trajectory and Ellipse Long", figsize=(16, 9), dpi=120)
    ax_trj_el_long = fig_trj_el_long.add_subplot(111)

    fig_trj_el_short = plt.figure("Trajectory and Ellipse Short", figsize=(16, 9), dpi=120)
    ax_trj_el_short = fig_trj_el_short.add_subplot(111)

    fig_trj_el2_longitudinal = plt.figure("Trajectory and Ellipse Longitudinal", figsize=(16, 9), dpi=120)
    ax_trj_el2_longitudinal = fig_trj_el2_longitudinal.add_subplot(111)

    fig_trj_el2_lateral = plt.figure("Trajectory and Ellipse Lateral", figsize=(16, 9), dpi=120)
    ax_trj_el2_lateral = fig_trj_el2_lateral.add_subplot(111)

    create_ellipse(pose_param, ax_trj_el_long)
    create_ellipse(pose_param, ax_trj_el_short)
    create_ellipse(pose_param, ax_trj_el2_longitudinal)
    create_ellipse(pose_param, ax_trj_el2_lateral)

    ax_trj_el_long.set_title("Trajectory and Ellipse Long")
    scatter_el_long = ax_trj_el_long.scatter(
        pose_param.df["x"], pose_param.df["y"], c=pose_param.df["ellipse_long"], vmin=0.0, cmap="jet"
    )
    fig_trj_el_long.colorbar(scatter_el_long, label="ellipse long[m]")
    ax_trj_el_long.set_xlabel("x[m]")
    ax_trj_el_long.set_ylabel("y[m]")
    ax_trj_el_long.set_aspect("equal")
    ax_trj_el_long.grid()
    fig_trj_el_long.savefig(output_dir + "/trj_el_long.png")

    ax_trj_el_short.set_title("Trajectory and Ellipse Short")
    scatter_el_short = ax_trj_el_short.scatter(
        pose_param.df["x"], pose_param.df["y"], c=pose_param.df["ellipse_short"], vmin=0.0, cmap="jet"
    )
    fig_trj_el_short.colorbar(scatter_el_short, label="ellipse short[m]")
    ax_trj_el_short.set_xlabel("x[m]")
    ax_trj_el_short.set_ylabel("y[m]")
    ax_trj_el_short.set_aspect("equal")
    ax_trj_el_short.grid()
    fig_trj_el_short.savefig(output_dir + "/trj_el_short.png")

    ax_trj_el2_longitudinal.set_title("Trajectory and Ellipse Longitudinal Direction")
    scatter_el_longitudinal = ax_trj_el2_longitudinal.scatter(
        pose_param.df["x"], pose_param.df["y"], c=pose_param.df["ellipse_longitudinal"], vmin=0.0, cmap="jet"
    )
    fig_trj_el2_longitudinal.colorbar(scatter_el_longitudinal, label="ellipse longitudinal[m]")
    ax_trj_el2_longitudinal.set_xlabel("x[m]")
    ax_trj_el2_longitudinal.set_ylabel("y[m]")
    ax_trj_el2_longitudinal.set_aspect("equal")
    ax_trj_el2_longitudinal.grid()
    fig_trj_el2_longitudinal.savefig(output_dir + "/trj_el_longitudinal.png")

    ax_trj_el2_lateral.set_title("Trajectory and Ellipse Lateral Direction")
    scatter_el_lateral = ax_trj_el2_lateral.scatter(
        pose_param.df["x"], pose_param.df["y"], c=pose_param.df["ellipse_lateral"], vmin=0.0, cmap="jet"
    )
    fig_trj_el2_lateral.colorbar(scatter_el_lateral, label="ellipse lateral[m]")
    ax_trj_el2_lateral.set_xlabel("x[m]")
    ax_trj_el2_lateral.set_ylabel("y[m]")
    ax_trj_el2_lateral.set_aspect("equal")
    ax_trj_el2_lateral.grid()
    fig_trj_el2_lateral.savefig(output_dir + "/trj_el_lateral.png")


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
    # trajectory nvtl
    if pose_empty == False and nvtl_empty == False:
        plot_trajectory_nvtl(pose_param, nvtl, output_dir)

    # trajectory tp
    if pose_empty == False and tp_empty == False:
        plot_trajectory_tp(pose_param, tp, output_dir)

    # nvtl
    if nvtl_empty == False:
        plot_nvtl(nvtl, output_dir)

    # tp
    if tp_empty == False:
        plot_tp(tp, output_dir)

    # execution time
    if exe_time_empty == False:
        plot_exe_time(exe_time, output_dir)

    # iteration
    if itr_empty == False:
        plot_itr(itr, output_dir)

    # plot ellipse
    if pose_empty == False:
        adjust.calc_ellipse(pose_param)
        plot_ellipse_ls(pose_param, output_dir)
        plot_ellipse_ll(pose_param, output_dir)
        plot_ellipse_trj(pose_param, output_dir)

    plt.show()