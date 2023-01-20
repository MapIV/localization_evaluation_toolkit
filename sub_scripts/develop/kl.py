import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# sys.path.append("../scripts")
from scripts.util import adjust, read_ros2bag, yaml_param


def calc_kl(ref_param, result_param):
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
        inv_result_cov = np.linalg.inv(result_cov)
        # KL = math.log(det_ref_cov / det_result_cov) + np.trace(np.dot(inv_ref_cov, result_cov)) - 2
        KL = math.log(det_result_cov / det_ref_cov) + np.trace(np.dot(inv_result_cov, ref_cov)) - 2
        kl_list.append(KL)
        if KL > 10:
            mismatch += 1
    return kl_list, mismatch


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

    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(ref_param, result_param) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(ref_param, result_param)
    print("Completed!!")

    print("Synchronizing time...", end="")
    ref_param.df, result_param.df = adjust.sync_time(ref_param, result_param, op_param)
    del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    print("Calculating KL divergence for 2D distributions...", end="")
    kl_list, mismatch = calc_kl(ref_param, result_param)

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
    plt.show()
