import math
import os
import sys

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
import yaml
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# sys.path.append("../scripts")
from scripts.util import adjust, read_ros2bag, yaml_param

import kl


def remove_cm(param, init_aray, converged_array):
    delnum = 0
    for i in range(len(init_aray)):
        if param.df["time"][i - delnum] != init_aray[i - delnum].time:
            del init_aray[i - delnum]
            del converged_array[i - delnum]
            delnum += 1


def ellipse_compare(
    i,
    ref_param,
    result_param,
    ref_init_aray,
    ref_converged_array,
    result_init_array,
    result_converged_array,
    kl_list,
    mismatch,
):
    ax_route.cla()
    ax_kl.cla()
    ax_ref_conv.cla()
    ax_result_conv.cla()

    # route
    ax_route.set_title("Route")
    ax_route.scatter(result_param.df["x"], result_param.df["y"], c="k", marker="o", s=5, zorder=1)
    ax_route.set_xlabel("x [m]")
    ax_route.set_ylabel("y [m]")
    ax_route.grid()

    # KL
    ax_kl.set_title("KL divergence")
    ax_kl.plot(ref_param.df["time"] - ref_param.df["time"][0], kl_list, marker="o", c="k", markersize=1, zorder=1)
    ax_kl.text(
        0.05, 1.03, "mismatch:{}/{}".format(mismatch, int(len(result_param.df))), fontsize=10, transform=ax_kl.transAxes
    )
    ax_kl.set_ylim(-1, max(kl_list) + 10)
    ax_kl.set_xlabel("time[s]")
    ax_kl.set_ylabel("KL divergence")
    ax_kl.grid()

    # convergence evaluation
    ax_ref_conv.set_title(ref_param.df["time"][i])
    ax_ref_conv.set_xlim(-3, 3)
    ax_ref_conv.set_ylim(-3, 3)
    ax_ref_conv.set_xlabel("x [m]")
    ax_ref_conv.set_ylabel("y [m]")
    ax_ref_conv.grid()
    ax_ref_conv.set_aspect("equal")

    # propsoed method
    ax_result_conv.set_title(result_param.df["time"][i])
    ax_result_conv.set_xlim(-3, 3)
    ax_result_conv.set_ylim(-3, 3)
    ax_result_conv.set_xlabel("x [m]")
    ax_result_conv.set_ylabel("y [m]")
    ax_result_conv.grid()
    ax_result_conv.set_aspect("equal")

    # route
    ax_route.scatter(result_param.df["x"][i], result_param.df["y"][i], c="r", marker="o", s=15, zorder=2)
    # KL
    ax_kl.scatter(ref_param.df["time"][i] - ref_param.df["time"][0], kl_list[i], c="r", marker="o", s=15, zorder=2)

    # convergence evaluation
    trans_ref_init_x = ref_init_aray[i].x - ref_param.df["x"][i]
    trans_ref_init_y = ref_init_aray[i].y - ref_param.df["y"][i]
    trans_ref_conv_x = ref_converged_array[i].x - ref_param.df["x"][i]
    trans_ref_conv_y = ref_converged_array[i].y - ref_param.df["y"][i]
    ax_ref_conv.scatter(trans_ref_init_x, trans_ref_init_y, c="r", marker="o", s=5, zorder=2)
    ax_ref_conv.scatter(trans_ref_conv_x, trans_ref_conv_y, c="k", marker="o", s=10, zorder=3)
    line_x = [trans_ref_init_x, trans_ref_conv_x]
    line_y = [trans_ref_init_y, trans_ref_conv_y]
    ax_ref_conv.plot(line_x, line_y, "g-", linewidth=0.2, zorder=1)
    e = patches.Ellipse(
        xy=(0, 0),
        width=ref_param.df["ellipse_long"][i] * 2,
        height=ref_param.df["ellipse_short"][i] * 2,
        angle=math.degrees(ref_param.df["ellipse_yaw"][i]),
        fill=False,
        ec="r",
        zorder=4,
    )
    ax_ref_conv.add_patch(e)

    # proposed method
    trans_result_init_x = result_init_array[i].x - result_param.df["x"][i]
    trans_result_init_y = result_init_array[i].y - result_param.df["y"][i]
    trans_result_conv_x = result_converged_array[i].x - result_param.df["x"][i]
    trans_result_conv_y = result_converged_array[i].y - result_param.df["y"][i]
    ax_result_conv.scatter(trans_result_init_x, trans_result_init_y, c="r", marker="o", s=5, zorder=2)
    ax_result_conv.scatter(trans_result_conv_x, trans_result_conv_y, c="k", marker="o", s=10, zorder=3)
    line_x = [trans_result_init_x, trans_result_conv_x]
    line_y = [trans_result_init_y, trans_result_conv_y]
    ax_result_conv.plot(line_x, line_y, "g-", linewidth=0.2, zorder=1)
    e = patches.Ellipse(
        xy=(0, 0),
        width=result_param.df["ellipse_long"][i] * 2,
        height=result_param.df["ellipse_short"][i] * 2,
        angle=math.degrees(result_param.df["ellipse_yaw"][i]),
        fill=False,
        ec="r",
        zorder=4,
    )
    ax_result_conv.add_patch(e)

    print("\r", str(i + 1) + " / " + str(len(ref_param.df)), end="")


if __name__ == "__main__":

    argv = sys.argv
    ref_path = argv[1]
    result_path = argv[2]
    config_path = argv[3]
    output_dir = argv[4]

    print("Loading yaml file ...", end="")
    with open(config_path, "r") as yml:
        config = yaml.safe_load(yml)
    ref_param = yaml_param.YamlParam()
    result_param = yaml_param.YamlParam()
    save_param = yaml_param.SaveParam()
    op_param = yaml_param.OpParam()
    adjust.input_yaml_ros2(config, ref_param, "Reference")
    adjust.set_tf(config, ref_param, "Reference")
    adjust.input_yaml_ros2(config, result_param, "Result")
    adjust.set_tf(config, result_param, "Result")
    adjust.input_save_param(config, save_param)
    adjust.input_op_param_ros2(config, op_param)
    op_param.display_ellipse = True
    print("Completed!!")

    print("Loading ros2 bag files ...", end="")
    read_ros2bag.read_ros2bag(ref_path, ref_param, op_param)
    read_ros2bag.read_ros2bag(result_path, result_param, op_param)
    print("Completed!!")

    ref_init_aray = []
    ref_converged_array = []
    result_init_array = []
    result_converged_array = []
    print("Loading convergence evaluation topic ...", end="")
    read_ros2bag.read_pose_array(ref_path, ref_init_aray, "/ndt_convergence_evaluator_initial_array")
    read_ros2bag.read_pose_array(ref_path, ref_converged_array, "/ndt_convergence_evaluator_result_array")
    read_ros2bag.read_pose_array(
        result_path, result_init_array, "/localization/pose_estimator/limited_convergence_evaluation_initial"
    )
    read_ros2bag.read_pose_array(
        result_path, result_converged_array, "/localization/pose_estimator/limited_convergence_evaluation"
    )
    print("Completed!!")

    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(ref_param, result_param) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(ref_param, result_param)
    print("Completed!!")

    print("Synchronizing time...", end="")
    ref_param.df, result_param.df = adjust.sync_time(ref_param, result_param, op_param)
    # del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    print("Calculating error ellipse ...", end="")
    adjust.calc_ellipse(ref_param)
    print("Completed!!")

    print("Calculating error ellipse ...", end="")
    adjust.calc_ellipse(result_param)
    print("Completed!!")

    print("Calculating KL divergence for 2D distributions...", end="")
    kl_list, mismatch = kl.calc_kl(ref_param, result_param)
    print("Completed!!")

    print("Remove the components...", end="")
    remove_cm(ref_param, ref_init_aray, ref_converged_array)
    remove_cm(result_param, result_init_array, result_converged_array)
    print("Completed!!")

    print(len(ref_param.df))
    print(len(result_param.df))

    print("Output convergence evaluation graph...", end="")

    conv_fig = plt.figure("Result", figsize=(16, 9), dpi=120)
    ax_route = conv_fig.add_subplot(223)
    ax_kl = conv_fig.add_subplot(221)
    ax_ref_conv = conv_fig.add_subplot(222)
    ax_result_conv = conv_fig.add_subplot(224)

    ani = animation.FuncAnimation(
        conv_fig,
        ellipse_compare,
        fargs=(
            ref_param,
            result_param,
            ref_init_aray,
            ref_converged_array,
            result_init_array,
            result_converged_array,
            kl_list,
            mismatch,
        ),
        interval=100,
        frames=len(ref_param.df),
    )
    # plt.show()
    ani.save(output_dir + "/animation.mp4", writer="ffmpeg")

    print("Completed!!")
