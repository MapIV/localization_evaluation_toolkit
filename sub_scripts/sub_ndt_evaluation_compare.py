import math
import os
import sys
import yaml


import matplotlib.pyplot as plt
plt.rcParams["font.size"] = 22
import numpy as np
import pandas as pd
import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# sys.path.append("../scripts")
from scripts.util import read_ros2bag
from scripts.deprecated import adjust, yaml_param


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
    bag_file = np.array([argv[1],argv[2]])
    config_path = argv[3]
    output_dir = argv[4]

    pose_ = np.array([BagParam(),BagParam()])
    nvtl_ = np.array([BagParam(),BagParam()])
    tp_ = np.array([BagParam(),BagParam()])
    exe_time_= np.array([BagParam(),BagParam()])
    itr_ = np.array([BagParam(),BagParam()])

    pose_empty=np.empty(2)
    nvtl_empty=np.empty(2)
    tp_empty=np.empty(2)
    exe_time_empty=np.empty(2)
    itr_empty=np.empty(2)

    print(itr_empty) 

    print("Loading yaml file ...", end="")
    with open(config_path, "r") as yml:
        config = yaml.safe_load(yml)
    ref_param = yaml_param.YamlParam()
    result_params = list()
    result_param = yaml_param.YamlParam()
    result_params.append(result_param)
    save_param = yaml_param.SaveParam()
    op_param = yaml_param.OpParam()
    adjust.input_yaml_ros2(config, ref_param, "Reference")
    adjust.set_tf(config, ref_param, "Reference")
    adjust.input_yaml_ros2(config, result_param, "Result")
    adjust.set_tf(config, result_param, "Result")
    adjust.input_save_param(config, save_param)
    adjust.input_op_param_ros2(config, op_param)
    print("Completed!!")

    for bag_index in range(len(pose_)):
        pose_[bag_index].topic = "/localization/pose_estimator/pose_with_covariance"
        read_ros2bag.read_pose(bag_file[bag_index], pose_[bag_index])
        pose_empty[bag_index] = pose_[bag_index].df_temp.empty

        nvtl_[bag_index].topic = "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
        read_ros2bag.read_unique(bag_file[bag_index], nvtl_[bag_index])
        nvtl_empty[bag_index] = nvtl_[bag_index].df_temp.empty

        tp_[bag_index].topic = "/localization/pose_estimator/transform_probability"
        read_ros2bag.read_unique(bag_file[bag_index], tp_[bag_index])
        tp_empty[bag_index] = tp_[bag_index].df_temp.empty

        exe_time_[bag_index].topic = "/localization/pose_estimator/exe_time_ms"
        read_ros2bag.read_unique(bag_file[bag_index], exe_time_[bag_index])
        exe_time_empty[bag_index] = exe_time_[bag_index].df_temp.empty

        itr_[bag_index].topic = "/localization/pose_estimator/iteration_num"
        read_ros2bag.read_unique(bag_file[bag_index], itr_[bag_index])
        itr_empty[bag_index] = itr_[bag_index].df_temp.empty

        path= output_dir+'/'+os.path.splitext(os.path.basename(bag_file[bag_index]))[0]
        os.makedirs(path, exist_ok = True)

        # Plot
        if pose_empty[bag_index] == False and nvtl_empty[bag_index] == False:
            for i in range(0, len(pose_[bag_index].df_temp)):
                search_sync = abs(nvtl_[bag_index].df_temp.iloc[:, 0] - pose_[bag_index].df_temp.iloc[i, 0])
                sync_id = search_sync.idxmin()
                nvtl_[bag_index].df = nvtl_[bag_index].df.append(nvtl_[bag_index].df_temp.iloc[sync_id, :], ignore_index=True)
            nvtl_[bag_index].df.reset_index(inplace=True, drop=True)

            #fig_trj_nvtl = plt.figure("Trajectory and NVTL", figsize=(16, 9), dpi=120)
            #ax_trj_nvtl = fig_trj_nvtl.add_subplot(111)
            fig_trj_nvtl, ax_trj_nvtl = plt.subplots(figsize=(16, 9), dpi=120)
            ax_trj_nvtl.set_title("Trajectory and NVTL")
            scatter_nvtl = ax_trj_nvtl.scatter(
                pose_[bag_index].df_temp["x"], pose_[bag_index].df_temp["y"], c=nvtl_[bag_index].df["data"], vmin=0.0, vmax=5, cmap="jet"
            )
            plt.colorbar(scatter_nvtl, label="NVTL")
            ax_trj_nvtl.set_xlabel("x[m]")
            ax_trj_nvtl.set_ylabel("y[m]")
            ax_trj_nvtl.set_aspect("equal")
            ax_trj_nvtl.grid()
            fig_trj_nvtl.savefig(path + "/trj_nvtl.png")

        if pose_empty[bag_index] == False and tp_empty[bag_index] == False:
            for i in range(0, len(pose_[bag_index].df_temp)):
                search_sync = abs(tp_[bag_index].df_temp.iloc[:, 0] - pose_[bag_index].df_temp.iloc[i, 0])
                sync_id = search_sync.idxmin()
                tp_[bag_index].df = tp_[bag_index].df.append(tp_[bag_index].df_temp.iloc[sync_id, :], ignore_index=True)
            tp_[bag_index].df.reset_index(inplace=True, drop=True)

            #fig_trj_tp = plt.figure("Trajectory and TP", figsize=(16, 9), dpi=120)
            #ax_trj_tp = fig_trj_tp.add_subplot(111)
            fig_trj_tp, ax_trj_tp = plt.subplots(figsize=(16, 9), dpi=120)
            ax_trj_tp.set_title("Trajectory and TP")
            scatter_tp = ax_trj_tp.scatter(
                pose_[bag_index].df_temp["x"], pose_[bag_index].df_temp["y"], c=tp_[bag_index].df["data"], vmin=0.0, vmax=7.0, cmap="jet"
            )
            plt.colorbar(scatter_tp, label="TP")
            ax_trj_tp.set_xlabel("x[m]")
            ax_trj_tp.set_ylabel("y[m]")
            ax_trj_tp.set_aspect("equal")
            ax_trj_tp.grid()
            fig_trj_tp.savefig(path + "/trj_tp.png")

        if nvtl_empty[bag_index] == False:
            #fig_nvtl = plt.figure("NVTL", figsize=(16, 9), dpi=120)
            #ax_nvtl = fig_nvtl.add_subplot(111)
            fig_nvtl, ax_nvtl = plt.subplots(figsize=(16, 9), dpi=120)
            ax_nvtl.set_title("NVTL")
            ax_nvtl.plot(
                nvtl_[bag_index].df_temp["time"] - nvtl_[bag_index].df_temp.at[0, "time"], nvtl_[bag_index].df_temp["data"], marker="o", c="b", markersize=2
            )
            ax_nvtl.set_xlabel("time[s]")
            ax_nvtl.set_ylabel("NVTL")
            ax_nvtl.set_ylim(0, 5)
            ax_nvtl.grid()
            fig_nvtl.savefig(path + "/nvtl.png")

        if tp_empty[bag_index] == False:
            #fig_tp = plt.figure("TP", figsize=(16, 9), dpi=120)
            #ax_tp = fig_tp.add_subplot(111)
            fig_tp, ax_tp = plt.subplots(figsize=(16, 9), dpi=120)
            ax_tp.set_title("TP")
            ax_tp.plot(
                tp_[bag_index].df_temp["time"] - tp_[bag_index].df_temp.at[0, "time"], tp_[bag_index].df_temp["data"], marker="o", c="b", markersize=2
            )
            ax_tp.set_xlabel("time[s]")
            ax_tp.set_ylabel("TP")
            ax_tp.set_ylim(0, 7)
            ax_tp.grid()
            fig_tp.savefig(path + "/tp.png")

        if exe_time_empty[bag_index] == False:
            #fig_exe_time = plt.figure("Execution Time", figsize=(16, 9), dpi=120)
            #ax_exe_time = fig_exe_time.add_subplot(111)
            fig_exe_time, ax_exe_time = plt.subplots(figsize=(16, 9), dpi=120)
            ax_exe_time.set_title("Execution Time")
            ax_exe_time.plot(
                exe_time_[bag_index].df_temp["time"] - exe_time_[bag_index].df_temp.at[0, "time"],
                exe_time_[bag_index].df_temp["data"],
                marker="o",
                c="b",
                markersize=2,
            )
            ax_exe_time.set_xlabel("time[s]")
            ax_exe_time.set_ylabel("Execution Time")
            ax_exe_time.set_ylim(0, 150)
            ax_exe_time.grid()
            fig_exe_time.savefig(path + "/exe_time.png")

        if itr_empty[bag_index] == False:
            #fig_itr = plt.figure("Iteration", figsize=(16, 9), dpi=120)
            #ax_itr = fig_itr.add_subplot(111)
            fig_itr, ax_itr = plt.subplots(figsize=(16, 9), dpi=120)
            ax_itr.set_title("Iteration")
            ax_itr.plot(
                itr_[bag_index].df_temp["time"] - itr_[bag_index].df_temp.at[0, "time"], itr_[bag_index].df_temp["data"], marker="o", c="b", markersize=2
            )
            ax_itr.set_xlabel("time[s]")
            ax_itr.set_ylabel("Iteration")
            ax_itr.grid()
            fig_itr.savefig(path + "/itr.png")

        
    # create compare graphs
    path= output_dir+'/compare'
    os.makedirs(path, exist_ok = True)

    color_list=["b", "r"]
    label_list=[os.path.splitext(os.path.basename(bag_file[0]))[0], os.path.splitext(os.path.basename(bag_file[1]))[0]]
    size_list=[8, 4]


    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(pose_[0], pose_[1]) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(pose_[0], pose_[1])
    print("Completed!!")

    print("Synchronizing time...", end="")
    pose_[0].df, pose_[1].df = adjust.sync_time(pose_[0], pose_[1], op_param)
    # del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    #fig_2d_trj = plt.figure("2D_Trajectory", figsize=(16, 9), dpi=120)
    #ax_2d_trj = fig_2d_trj.add_subplot(111)
    fig_2d_trj, ax_2d_trj = plt.subplots(figsize=(16, 9), dpi=120)
    ax_2d_trj.set_title("2D Trajectory")
    for i in range(len(pose_)):
        ax_2d_trj.scatter(pose_[i].df["x"], pose_[i].df["y"], s=size_list[i],color=color_list[i], label=label_list[i])

    ax_2d_trj.plot([pose_[0].df["x"], pose_[1].df["x"]], [pose_[0].df["y"], pose_[1].df["y"]], "-", linewidth=0.2, zorder=1)
    ax_2d_trj.set_xlabel("x[m]")
    ax_2d_trj.set_ylabel("y[m]")
    # ax_2d_trj.tick_params(labelsize=save_param.ticks_font_size)
    ax_2d_trj.legend()
    ax_2d_trj.set_aspect("equal")
    ax_2d_trj.grid()

    fig_2d_trj.savefig(path + "/2DTrajectory.png")

    # print("Synchronizing time...", end="")
    # ref_param.df, result_param.df = adjust.sync_time(ref_param, result_param, op_param)
    # del ref_param.df_temp, result_param.df_temp
    # print("Completed!!")

    # Plot
    # if pose_empty[0] == False and nvtl_empty[0] == False and pose_empty[1] == False and nvtl_empty[1] == False:
    #     pose_diff=
    #     for i in range(0, len(pose_[0].df_temp)):
    #         search_sync = abs(nvtl_[0].df_temp.iloc[:, 0] - pose_[i].df_temp.iloc[i, 0])
    #         sync_id = search_sync.idxmin()
    #         nvtl_[i].df = nvtl_[i].df.append(nvtl_[i].df_temp.iloc[sync_id, :], ignore_index=True)
    #     nvtl_[i].df.reset_index(inplace=True, drop=True)

    #     fig_trj_nvtl = plt.figure("Trajectory and NVTL", figsize=(16, 9), dpi=120)
    #     ax_trj_nvtl = fig_trj_nvtl.add_subplot(111)
    #     ax_trj_nvtl.set_title("Trajectory and NVTL")
    #     scatter_nvtl = ax_trj_nvtl.scatter(
    #         pose_[i].df_temp["x"], pose_[i].df_temp["y"], c=nvtl_[i].df["data"], vmin=0.0, vmax=5, cmap="jet"
    #     )
    #     plt.colorbar(scatter_nvtl, label="NVTL")
    #     ax_trj_nvtl.set_xlabel("x[m]")
    #     ax_trj_nvtl.set_ylabel("y[m]")
    #     ax_trj_nvtl.set_aspect("equal")
    #     ax_trj_nvtl.grid()
    #     fig_trj_nvtl.savefig(path + "/trj_nvtl.png")

    # if pose_empty[i] == False and tp_empty[i] == False:
    #     for i in range(0, len(pose_[i].df_temp)):
    #         search_sync = abs(tp_[i].df_temp.iloc[:, 0] - pose_[i].df_temp.iloc[i, 0])
    #         sync_id = search_sync.idxmin()
    #         tp_[i].df = tp_[i].df.append(tp_[i].df_temp.iloc[sync_id, :], ignore_index=True)
    #     tp_[i].df.reset_index(inplace=True, drop=True)

    #     fig_trj_tp = plt.figure("Trajectory and TP", figsize=(16, 9), dpi=120)
    #     ax_trj_tp = fig_trj_tp.add_subplot(111)
    #     ax_trj_tp.set_title("Trajectory and TP")
    #     scatter_tp = ax_trj_tp.scatter(
    #         pose_[i].df_temp["x"], pose_[i].df_temp["y"], c=tp_.df["data"], vmin=0.0, vmax=7.0, cmap="jet"
    #     )
    #     plt.colorbar(scatter_tp, label="TP")
    #     ax_trj_tp.set_xlabel("x[m]")
    #     ax_trj_tp.set_ylabel("y[m]")
    #     ax_trj_tp.set_aspect("equal")
    #     ax_trj_tp.grid()
    #     fig_trj_tp.savefig(path + "/trj_tp.png")
    
    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(nvtl_[0], nvtl_[1]) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(nvtl_[0], nvtl_[1])
    print("Completed!!")

    print("Synchronizing time...", end="")
    nvtl_[0].df, nvtl_[1].df = adjust.sync_time(nvtl_[0], nvtl_[1], op_param)
    # del ref_param.df_temp, result_param.df_temp
    print("Completed!!")


    if nvtl_empty[0] == False and nvtl_empty[1] == False:
        #fig_nvtl_c = plt.figure("NVTL", figsize=(16, 9), dpi=120)
        #ax_nvtl = fig_nvtl_c.add_subplot(111)
        fig_nvtl, ax_nvtl = plt.subplots(figsize=(16, 9), dpi=120)
        ax_nvtl.set_title("NVTL")

        for i in range(len(nvtl_)):
            ax_nvtl.plot(
                nvtl_[i].df["time"] - nvtl_[i].df.at[0, "time"], nvtl_[i].df["data"], marker="o", c=color_list[i], markersize=2, label=label_list[i]
            )
        ax_nvtl.set_xlabel("time[s]")
        ax_nvtl.set_ylabel("NVTL")
        ax_nvtl.set_ylim(0, 5)
        ax_nvtl.grid()
        ax_nvtl.legend()
        fig_nvtl.savefig(path + "/nvtl.png")

        #fig_nvtl_diff = plt.figure("NVTLDiff", figsize=(16, 9), dpi=120)
        #ax_nvtl_diff = fig_nvtl_diff.add_subplot(111)
        fig_nvtl_diff, ax_nvtl_diff = plt.subplots(figsize=(16, 9), dpi=120)
        ax_nvtl_diff.set_title("NVTL DIFF")
        
        ax_nvtl_diff.plot(
            nvtl_[0].df["time"] - nvtl_[i].df.at[0, "time"], nvtl_[1].df["data"]-nvtl_[0].df["data"], marker="o", c="b", markersize=2
        )
        ax_nvtl_diff.set_xlabel("time[s]")
        ax_nvtl_diff.set_ylabel("NVTL DIFF")
        ax_nvtl_diff.set_ylim(-1, 1)
        ax_nvtl_diff.grid()
        fig_nvtl_diff.savefig(path + "/nvtl-diff.png")
    
    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(tp_[0], tp_[1]) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(tp_[0], tp_[1])
    print("Completed!!")

    print("Synchronizing time...", end="")
    tp_[0].df, tp_[1].df = adjust.sync_time(tp_[0], tp_[1], op_param)
    # del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    if tp_empty[0] == False and tp_empty[1] == False:
        #fig_tp = plt.figure("TP", figsize=(16, 9), dpi=120)
        #ax_tp = fig_tp.add_subplot(111)
        fig_tp, ax_tp = plt.subplots(figsize=(16, 9), dpi=120)
        ax_tp.set_title("TP")

        for i in range(len(tp_)):
            ax_tp.plot(
                tp_[i].df["time"] - tp_[i].df.at[0, "time"], tp_[i].df["data"], marker="o", c=color_list[i], markersize=2, label=label_list[i]
            )
        ax_tp.set_xlabel("time[s]")
        ax_tp.set_ylabel("TP")
        ax_tp.set_ylim(0, 7)
        ax_tp.grid()
        ax_tp.legend()
        fig_tp.savefig(path + "/tp.png")

        #fig_tp_diff = plt.figure("TP DIFF", figsize=(16, 9), dpi=120)
        #ax_tp_diff = fig_tp_diff.add_subplot(111)
        fig_tp_diff, ax_tp_diff = plt.subplots(figsize=(16, 9), dpi=120)
        ax_tp_diff.set_title("TP DIFF")

        ax_tp_diff.plot(
            tp_[0].df["time"] - tp_[0].df.at[0, "time"], tp_[1].df["data"]-tp_[0].df["data"], marker="o", c="b", markersize=2
        )

        ax_tp_diff.set_xlabel("time[s]")
        ax_tp_diff.set_ylabel("TP Diff")
        ax_tp_diff.set_ylim(-1, 1)
        ax_tp_diff.grid()
        ax_tp_diff.legend()
        fig_tp_diff.savefig(path + "/tp_diff.png")

    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(exe_time_[0], exe_time_[1]) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(exe_time_[0], exe_time_[1])
    print("Completed!!")

    print("Synchronizing time...", end="")
    exe_time_[0].df, exe_time_[1].df = adjust.sync_time(exe_time_[0], exe_time_[1], op_param)
    # del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    if exe_time_empty[0] == False and exe_time_empty[1] == False:
        #fig_exe_time = plt.figure("Execution Time", figsize=(16, 9), dpi=120)
        #ax_exe_time = fig_exe_time.add_subplot(111)
        fig_exe_time, ax_exe_time = plt.subplots(figsize=(16, 9), dpi=120)
        ax_exe_time.set_title("Execution Time")


        for i in range(len(exe_time_)):
            ax_exe_time.plot(
                exe_time_[i].df["time"] - exe_time_[i].df.at[0, "time"],
                exe_time_[i].df["data"],
                marker="o",
                c=color_list[i],
                markersize=2,
                label=label_list[i],
            )
        ax_exe_time.set_xlabel("time[s]")
        ax_exe_time.set_ylabel("Execution Time")
        ax_exe_time.set_ylim(0, 150)
        ax_exe_time.grid()
        ax_exe_time.legend()
        fig_exe_time.savefig(path + "/exe_time.png")

        #fig_exe_time_diff = plt.figure("Execution Time Diff", figsize=(16, 9), dpi=120)
        #ax_exe_time_diff = fig_exe_time_diff.add_subplot(111)
        fig_exe_time_diff, ax_exe_time_diff = plt.subplots(figsize=(16, 9), dpi=120)
        ax_exe_time_diff.set_title("Execution Time Diff")

        ax_exe_time_diff.plot(
            exe_time_[0].df["time"] - exe_time_[0].df.at[0, "time"],
            exe_time_[1].df["data"]-exe_time_[0].df["data"],
            marker="o",
            c="b",
            markersize=2,
        )
        ax_exe_time_diff.set_xlabel("time[s]")
        ax_exe_time_diff.set_ylabel("Execution Time Diff")
        ax_exe_time_diff.set_ylim(-100, 100)
        ax_exe_time_diff.grid()
        ax_exe_time_diff.legend()
        fig_exe_time_diff.savefig(path + "/exe_time_diff.png")

    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(itr_[0], itr_[1]) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(itr_[0], itr_[1])
    print("Completed!!")

    print("Synchronizing time...", end="")
    itr_[0].df, itr_[1].df = adjust.sync_time(itr_[0], itr_[1], op_param)
    # del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    if itr_empty[i] == False:
        #fig_itr = plt.figure("Iteration", figsize=(16, 9), dpi=120)
        #ax_itr = fig_itr.add_subplot(111)
        fig_itr, ax_itr = plt.subplots(figsize=(16, 9), dpi=120)
        ax_itr.set_title("Iteration")
        for i in range(len(itr_)):
            ax_itr.plot(
                itr_[i].df_temp["time"] - itr_[i].df_temp.at[0, "time"], itr_[i].df_temp["data"], marker="o", c=color_list[i], markersize=2, label=label_list[i]
            )
        ax_itr.set_xlabel("time[s]")
        ax_itr.set_ylabel("Iteration")
        ax_itr.grid()
        ax_itr.legend()
        fig_itr.savefig(path + "/itr.png")


        #fig_itr_diff = plt.figure("Iteration Diff", figsize=(16, 9), dpi=120)
        #ax_itr_diff = fig_itr_diff.add_subplot(111)
        fig_itr_diff, ax_itr_diff = plt.subplots(figsize=(16, 9), dpi=120)
        ax_itr_diff.set_title("Iteration Diff")
        ax_itr_diff.plot(
            itr_[0].df["time"] - itr_[0].df.at[0, "time"], itr_[1].df["data"]-itr_[0].df["data"], marker="o", c="b", markersize=2,
        )
        ax_itr_diff.set_xlabel("time[s]")
        ax_itr_diff.set_ylabel("Iteration Diff")
        ax_itr_diff.grid()
        ax_itr_diff.legend()
        fig_itr_diff.savefig(path + "/itr_diff.png")



    plt.show()
