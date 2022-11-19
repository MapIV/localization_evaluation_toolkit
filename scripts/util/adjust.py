import math

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R


def input_yaml_param(config, param, item):
    param.label = config[item]["label"]
    param.separate_time_stamp = config[item]["separate_time_stamp"]
    param.stamp_column = config[item]["stamp_column"]
    param.secs_stamp_column = config[item]["secs_stamp_column"]
    param.nsecs_stamp_column = config[item]["nsecs_stamp_column"]
    param.x_column = config[item]["x_column"]
    param.y_column = config[item]["y_column"]
    param.z_column = config[item]["z_column"]
    param.use_quaternion = config[item]["use_quaternion"]
    param.ori_x_column = config[item]["ori_x_column"]
    param.ori_y_column = config[item]["ori_y_column"]
    param.ori_z_column = config[item]["ori_z_column"]
    param.ori_w_column = config[item]["ori_w_column"]
    param.use_radian = config[item]["use_radian"]
    param.roll_column = config[item]["roll_column"]
    param.pitch_column = config[item]["pitch_column"]
    param.yaw_column = config[item]["yaw_column"]


def set_tf(config, param, item):
    param.tf_time = config[item]["tf_time"]
    param.tf_x = config[item]["tf_x"]
    param.tf_y = config[item]["tf_y"]
    param.tf_z = config[item]["tf_z"]
    param.tf_roll = config[item]["tf_roll"]
    param.tf_pitch = config[item]["tf_pitch"]
    param.tf_yaw = config[item]["tf_yaw"]
    param.inv_roll = config[item]["inv_roll"]
    param.inv_pitch = config[item]["inv_pitch"]
    param.inv_yaw = config[item]["inv_yaw"]


def input_yaml_ros2(config, param, item):
    param.topic = config[item]["topic_name"]
    param.bag_id = config[item]["storage_id"]
    param.bag_format = config[item]["serialization_format"]


def input_save_param(config, save_param):
    save_param.axis_type = config["axis_type"]
    save_param.display_radian = config["use_radian"]
    save_param.progress_info = config["progress_info"]
    save_param.interval = config["interval"]
    save_param.title_font_size = config["title_font_size"]
    save_param.label_font_size = config["label_font_size"]
    save_param.ticks_font_size = config["ticks_font_size"]
    save_param.save_figures = config["save_figures"]
    save_param.save_extension_type = config["save_extension_type"]
    save_param.save_dataframe = config["save_dataframe"]


def input_op_param(config, op_param):
    op_param.use_lerp = config["use_lerp"]
    op_param.display_ellipse = config["display_ellipse"]
    if op_param.display_ellipse == True:
        op_param.covariance_xx_column = config["covariance_xx_column"]
        op_param.covariance_xy_column = config["covariance_xy_column"]
        op_param.covariance_yx_column = config["covariance_yx_column"]
        op_param.covariance_yy_column = config["covariance_yy_column"]


def input_op_param_ros2(config, op_param):
    op_param.use_lerp = config["use_lerp"]
    op_param.display_ellipse = config["display_ellipse"]


def unit_adjust(param, df_org):
    # Time
    if param.separate_time_stamp == True:
        param.df_temp["time"] = (
            df_org.iloc[:, param.secs_stamp_column] + df_org.iloc[:, param.nsecs_stamp_column] / 10**9 + param.tf_time
        )
    else:
        if len(str(int(df_org.iloc[0, param.stamp_column]))) > 10:
            param.df_temp["time"] = df_org.iloc[:, param.stamp_column] / 10**9 + param.tf_time
        else:
            param.df_temp["time"] = df_org.iloc[:, param.stamp_column] + param.tf_time

    # Position
    param.df_temp["x"] = df_org.iloc[:, param.x_column] + param.tf_x
    param.df_temp["y"] = df_org.iloc[:, param.y_column] + param.tf_y
    param.df_temp["z"] = df_org.iloc[:, param.z_column] + param.tf_z

    # Rotation
    if param.use_quaternion == True:
        for i in range(len(df_org)):
            ref_q_temp = [
                df_org.iloc[i, param.ori_x_column],
                df_org.iloc[i, param.ori_y_column],
                df_org.iloc[i, param.ori_z_column],
                df_org.iloc[i, param.ori_w_column],
            ]
            ref_e_temp = R.from_quat([ref_q_temp[0], ref_q_temp[1], ref_q_temp[2], ref_q_temp[3]])
            param.df_temp.at[i, "roll"] = (
                ref_e_temp.as_euler("ZYX", degrees=False)[2] + param.tf_roll
            ) * param.inv_roll
            param.df_temp.at[i, "pitch"] = (
                ref_e_temp.as_euler("ZYX", degrees=False)[1] + param.tf_pitch
            ) * param.inv_pitch
            param.df_temp.at[i, "yaw"] = (ref_e_temp.as_euler("ZYX", degrees=False)[0] + param.tf_yaw) * param.inv_yaw
    elif param.use_quaternion == False and param.use_radian == True:
        param.df_temp["roll"] = (df_org.iloc[:, param.roll_column] + param.tf_roll) * param.inv_roll
        param.df_temp["pitch"] = (df_org.iloc[:, param.pitch_column] + param.tf_pitch) * param.inv_pitch
        param.df_temp["yaw"] = (df_org.iloc[:, param.yaw_column] + param.tf_yaw) * param.inv_yaw
    elif param.use_quaternion == False and param.use_radian == False:
        param.df_temp["roll"] = (df_org.iloc[:, param.roll_column] * math.pi / 180 + param.tf_roll) * param.inv_roll
        param.df_temp["pitch"] = (df_org.iloc[:, param.pitch_column] * math.pi / 180 + param.tf_pitch) * param.inv_pitch
        param.df_temp["yaw"] = (df_org.iloc[:, param.yaw_column] * math.pi / 180 + param.tf_yaw) * param.inv_yaw


def add_covariance(param, df_org, op_param):
    param.df_temp["cov_xx"] = df_org.iloc[:, op_param.covariance_xx_column]
    param.df_temp["cov_xy"] = df_org.iloc[:, op_param.covariance_xy_column]
    param.df_temp["cov_yx"] = df_org.iloc[:, op_param.covariance_yx_column]
    param.df_temp["cov_yy"] = df_org.iloc[:, op_param.covariance_yy_column]


def adjust_start_time(ref_param, result_param):
    if (
        ref_param.df_temp.at[0, "time"] < result_param.df_temp.at[0, "time"]
        and ref_param.df_temp["time"].iloc[-1] > result_param.df_temp.at[0, "time"]
    ):
        search_ref_start = abs(ref_param.df_temp["time"] - result_param.df_temp.at[0, "time"])
        ref_start_time_index = search_ref_start.idxmin()
        ref_param.df_temp.drop(range(0, ref_start_time_index), inplace=True)
        ref_param.df_temp.reset_index(inplace=True, drop=True)
    elif (
        ref_param.df_temp.at[0, "time"] > result_param.df_temp.at[0, "time"]
        and ref_param.df_temp.at[0, "time"] < result_param.df_temp["time"].iloc[-1]
    ):
        search_result_start = abs(result_param.df_temp["time"] - ref_param.df_temp.at[0, "time"])
        result_start_time_index = search_result_start.idxmin()
        result_param.df_temp.drop(range(0, result_start_time_index), inplace=True)
        result_param.df_temp.reset_index(inplace=True, drop=True)
    elif ref_param.df_temp.at[0, "time"] == result_param.df_temp.at[0, "time"]:
        pass
    else:
        print("[ERROR]: Reference time stamp and Result time stamp do not overlap")
        return -1


def adjust_end_time(ref_param, result_param):
    if ref_param.df_temp["time"].iloc[-1] > result_param.df_temp["time"].iloc[-1]:
        search_ref_end = abs(ref_param.df_temp["time"] - result_param.df_temp["time"].iloc[-1])
        ref_end_time_index = search_ref_end.idxmin()
        ref_param.df_temp.drop(range(ref_end_time_index + 1, len(ref_param.df_temp.index)), inplace=True)
        ref_param.df_temp.reset_index(inplace=True, drop=True)
    elif ref_param.df_temp["time"].iloc[-1] < result_param.df_temp["time"].iloc[-1]:
        search_result_end = abs(result_param.df_temp["time"] - ref_param.df_temp["time"].iloc[-1])
        result_end_time_index = search_result_end.idxmin()
        result_param.df_temp.drop(range(result_end_time_index + 1, len(result_param.df_temp.index)), inplace=True)
        result_param.df_temp.reset_index(inplace=True, drop=True)


def sync_time(ref_param, result_param, op_param):
    before_sync = -1
    before_min_time = 0
    if len(ref_param.df_temp.index) >= len(result_param.df_temp.index):
        sync_ref_df = pd.DataFrame(columns=ref_param.df_temp.columns)
        for i in range(0, len(result_param.df_temp)):
            search_sync_ref_time = abs(ref_param.df_temp["time"] - result_param.df_temp.at[i, "time"])
            sync_ref_id = search_sync_ref_time.idxmin()
            min_ref_time = search_sync_ref_time.min()
            if sync_ref_id == before_sync:
                if min_ref_time < before_min_time:
                    sync_ref_df.drop(sync_ref_df.index[-1], inplace=True)
                    result_param.df_temp.drop(i - 1, inplace=True)
                elif min_ref_time > before_min_time:
                    result_param.df_temp.drop(i, inplace=True)
                    continue
            if op_param.use_lerp == True:
                lerp(ref_param.df_temp, sync_ref_id, min_ref_time, result_param.df_temp.at[i, "time"])
            sync_ref_df = pd.concat([sync_ref_df, ref_param.df_temp.iloc[[sync_ref_id]]], ignore_index=True)
            before_sync = sync_ref_id
            before_min_time = min_ref_time
        result_param.df_temp.reset_index(inplace=True, drop=True)
        del search_sync_ref_time, before_min_time
        return sync_ref_df, result_param.df_temp
    else:
        sync_result_df = pd.DataFrame(columns=result_param.df_temp.columns)
        ref_param.df = ref_param.df_temp.copy()
        for i in range(0, len(ref_param.df)):
            search_sync_result_time = abs(result_param.df_temp["time"] - ref_param.df.at[i, "time"])
            sync_result_id = search_sync_result_time.idxmin()
            min_result_time = search_sync_result_time.min()
            if sync_result_id == before_sync:
                if min_result_time < before_min_time:
                    sync_result_df.drop(sync_result_df.index[-1], inplace=True)
                    ref_param.df.drop(i - 1, inplace=True)
                elif min_result_time > before_min_time:
                    ref_param.df.drop(i, inplace=True)
                    continue
            if op_param.use_lerp == True:
                lerp(ref_param.df_temp, i, min_result_time, result_param.df_temp.at[sync_result_id, "time"])
            # sync_result_df = sync_result_df.append(result_param.df_temp.iloc[sync_result_id, :], ignore_index=True)
            sync_result_df = pd.concat([sync_result_df, result_param.df_temp.iloc[[sync_result_id]]], ignore_index=True)
            before_sync = sync_result_id
            before_min_time = min_result_time
        ref_param.df.reset_index(inplace=True, drop=True)
        del search_sync_result_time, before_min_time
        return ref_param.df, sync_result_df


def lerp(target_param, sync_id, min_ref_time, result_time):
    tar_sync_time = target_param.at[sync_id, "time"]
    if tar_sync_time > result_time:
        if sync_id == 0:
            return
        delta_t = tar_sync_time - target_param.at[sync_id - 1, "time"]
        if delta_t == 0:
            return
        tar_xbefore = target_param.at[sync_id - 1, "x"]
        tar_ybefore = target_param.at[sync_id - 1, "y"]
        tar_zbefore = target_param.at[sync_id - 1, "z"]
        target_param.at[sync_id, "x"] -= (target_param.at[sync_id, "x"] - tar_xbefore) * min_ref_time / (delta_t)
        target_param.at[sync_id, "y"] -= (target_param.at[sync_id, "y"] - tar_ybefore) * min_ref_time / (delta_t)
        target_param.at[sync_id, "z"] -= (target_param.at[sync_id, "z"] - tar_zbefore) * min_ref_time / (delta_t)
    elif tar_sync_time < result_time:
        if sync_id + 1 == len(target_param):
            return
        delta_t = target_param.at[sync_id + 1, "time"] - tar_sync_time
        if delta_t == 0:
            return
        tar_xafter = target_param.at[sync_id + 1, "x"]
        tar_yafter = target_param.at[sync_id + 1, "y"]
        tar_zafter = target_param.at[sync_id + 1, "z"]
        target_param.at[sync_id, "x"] += (tar_xafter - target_param.at[sync_id, "x"]) * min_ref_time / (delta_t)
        target_param.at[sync_id, "y"] += (tar_yafter - target_param.at[sync_id, "y"]) * min_ref_time / (delta_t)
        target_param.at[sync_id, "z"] += (tar_zafter - target_param.at[sync_id, "z"]) * min_ref_time / (delta_t)


def calc_ellipse(param):
    ellipse_dict = {
        "ellipse_long": [],
        "ellipse_short": [],
        "ellipse_yaw": [],
        "ellipse_longitudinal": [],
        "ellipse_lateral": [],
    }
    for i in range(len(param.df)):
        covariance = np.array(
            [[param.df.at[i, "cov_xx"], param.df.at[i, "cov_xy"]], [param.df.at[i, "cov_yx"], param.df.at[i, "cov_yy"]]]
        )
        eig_vals, eig_vec = np.linalg.eig(covariance)
        ellipse_long = math.sqrt(eig_vals[0] * 9.21)
        ellipse_short = math.sqrt(eig_vals[1] * 9.21)
        ellipse_yaw = round(math.atan2(eig_vec[1, 0], eig_vec[0, 0]), 4)
        th = ellipse_yaw - param.df.at[i, "yaw"]
        ellipse_dict["ellipse_long"].append(ellipse_long)
        ellipse_dict["ellipse_short"].append(ellipse_short)
        ellipse_dict["ellipse_yaw"].append(ellipse_yaw)
        ellipse_dict["ellipse_lateral"].append(
            math.sqrt(pow(ellipse_long * math.sin(th), 2) + pow(ellipse_short * math.cos(th), 2))
        )
        ellipse_dict["ellipse_longitudinal"].append(
            math.sqrt(
                pow(ellipse_long * math.sin(th + math.pi / 2), 2) + pow(ellipse_short * math.cos(th + math.pi / 2), 2)
            )
        )

    param.df = param.df.assign(
        ellipse_long=ellipse_dict["ellipse_long"],
        ellipse_short=ellipse_dict["ellipse_short"],
        ellipse_yaw=ellipse_dict["ellipse_yaw"],
        ellipse_lateral=ellipse_dict["ellipse_lateral"],
        ellipse_longitudinal=ellipse_dict["ellipse_longitudinal"],
    )
