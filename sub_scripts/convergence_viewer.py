import math
import os
import sys

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


class FrameCM:
    def __init__(self):
        self.time = ""
        self.x = []
        self.y = []
        self.z = []
        self.roll = []
        self.pitch = []
        self.yaw = []


def read_ce_topic(bag_file, frame_array, topic_name):
    bag_path = str(bag_file)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        fram_cm = FrameCM()

        fram_cm.time = (msg.header.stamp.sec) + (msg.header.stamp.nanosec) / 10**9
        for i in range(len(msg.poses)):
            q_temp = [
                msg.poses[i].orientation.x,
                msg.poses[i].orientation.y,
                msg.poses[i].orientation.z,
                msg.poses[i].orientation.w,
            ]
            e_temp = R.from_quat([q_temp[0], q_temp[1], q_temp[2], q_temp[3]])
            fram_cm.x.append(msg.poses[i].position.x)
            fram_cm.y.append(msg.poses[i].position.y)
            fram_cm.z.append(msg.poses[i].position.z)
            fram_cm.roll.append(e_temp.as_euler("ZYX", degrees=False)[2])
            fram_cm.pitch.append(e_temp.as_euler("ZYX", degrees=False)[1])
            fram_cm.yaw.append(e_temp.as_euler("ZYX", degrees=False)[0])
        frame_array.append(fram_cm)


def calc_kl(ref_param, result_param, output_dir):
    kl_list = []
    mismatch = 0
    for i in range(len(result_param.df)):

        ref_cov = np.array(
            [
                [ref_param.df.at[i, "cov_xx"], ref_param.df.at[i, "cov_xy"]],
                [ref_param.df.at[i, "cov_yx"], ref_param.df.at[i, "cov_yy"]],
            ]
        )
        result_cov = np.array(
            [
                [result_param.df.at[i, "cov_xx"], result_param.df.at[i, "cov_xy"]],
                [result_param.df.at[i, "cov_yx"], result_param.df.at[i, "cov_yy"]],
            ]
        )
        det_ref_cov = np.linalg.det(ref_cov)
        det_result_cov = np.linalg.det(result_cov)
        inv_ref_cov = np.linalg.inv(ref_cov)
        KL = math.log(det_ref_cov / det_result_cov) + np.trace(np.dot(inv_ref_cov, result_cov)) - 2
        kl_list.append(KL)
        if KL > 10:
            mismatch += 1

    fig_kl = plt.figure("KL divergence", figsize=(16, 9), dpi=120)
    ax_kl = fig_kl.add_subplot(111)
    ax_kl.set_title("KL divergence")
    ax_kl.plot(ref_param.df["time"] - ref_param.df.at[0, "time"], kl_list, marker="o", c="b", markersize=2)
    ax_kl.text(
        0.1, 1.05, "mismatch:{}/{}".format(mismatch, int(len(result_param.df))), fontsize=15, transform=ax_kl.transAxes
    )
    ax_kl.set_xlabel("time[s]")
    ax_kl.set_ylabel("KL divergence")
    ax_kl.grid()
    fig_kl.savefig(output_dir + "/kl.png")


def remove_cm(param, init_aray, converged_array):
    delnum = 0
    for i in range(len(init_aray)):
        if param.df["time"][i - delnum] != init_aray[i - delnum].time:
            del init_aray[i - delnum]
            del converged_array[i - delnum]
            delnum += 1


def ellipse_compare(
    ref_param, result_param, ref_init_aray, ref_converged_array, result_init_array, result_converged_array, output_dir
):
    conv_fig = plt.figure("Result", figsize=(16, 9), dpi=120)
    ax_ref_conv = conv_fig.add_subplot(121)
    ax_result_conv = conv_fig.add_subplot(122)
    for i in range(len(ref_param.df)):
        ax_ref_conv.cla()
        ax_result_conv.cla()
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
        ax_ref_conv.set_xlim(-3, 3)
        ax_ref_conv.set_ylim(-3, 3)
        ax_ref_conv.set_xlabel("x [m]")
        ax_ref_conv.set_ylabel("y [m]")
        ax_ref_conv.grid()
        ax_ref_conv.set_aspect("equal")

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
        ax_result_conv.set_xlim(-3, 3)
        ax_result_conv.set_ylim(-3, 3)
        ax_result_conv.set_xlabel("x [m]")
        ax_result_conv.set_ylabel("y [m]")
        ax_result_conv.grid()
        ax_result_conv.set_aspect("equal")

        conv_fig.savefig(output_dir + "/" + str(i) + ".png")


if __name__ == "__main__":

    argv = sys.argv
    ref_path = argv[1]
    result_path = argv[2]
    config_path = argv[3]
    output_dir = argv[4]

    # ref_path = "/home/koki/01_dataset/xx1/osaki/0922_result/test/test_nce_bag/rosbag2_2022_09_27-14_36_29/rosbag2_2022_09_27-14_36_29_0.db3"
    # result_path = "/home/koki/01_dataset/xx1/osaki/0922_result/test/test_prop_bag/rosbag2_2022_09_23-17_13_30/rosbag2_2022_09_23-17_13_30_0.db3"
    # config_path = "/home/koki/01_dataset/xx1/osaki/0922_result/conv_graph.yaml"
    # output_dir = "/home/koki/01_dataset/xx1/osaki/0922_result/test/test_cov_graph_short"
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
    calc_kl(ref_param, result_param, output_dir)
    print("Completed!!")

    print("Remove the components...", end="")
    remove_cm(ref_param, ref_init_aray, ref_converged_array)
    remove_cm(result_param, result_init_array, result_converged_array)
    print("Completed!!")

    print(len(ref_param.df))
    print(len(result_param.df))

    print("Output convergence evaluation graph...", end="")
    ellipse_compare(
        ref_param,
        result_param,
        ref_init_aray,
        ref_converged_array,
        result_init_array,
        result_converged_array,
        output_dir,
    )
    print("Completed!!")
