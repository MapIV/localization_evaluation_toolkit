# csv data packer

import math
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from typing import Tuple

from util.twist.configer import DataParam, OptParam

class DataPack:
    def __init__(self, ref_param: DataParam, twist_param, opt_param: OptParam) -> None:
        # basic info
        self.ref_label = ref_param.label
        self.twist_label = twist_param.label
        # read data
        self.ref_data = self.read_ref_data(ref_param)
        self.twist_data = self.read_twist_data(twist_param)
        # sync time
        self.sync_time(opt_param)
        # pre-calculation
        self.ref_data = self.ref_data.assign(**self.calc_distance(self.ref_data))
        self.ref_data = self.ref_data.assign(**self.calc_elapsed(self.ref_data))
        self.dr_error, self.dr_traj = self.get_dr_twist(opt_param)
        self.dr_rate = self.get_dr_rate(opt_param)

    def sync_time(self, opt_param: OptParam) -> None:
        sync_index = np.zeros(len(self.twist_data["time"]))

        set_target_index = []
        set_ref_index = []

        if self.ref_data["time"][0] >  self.twist_data["time"][0]:
            self.twist_data = self.twist_data[self.twist_data["time"] > self.ref_data["time"][0] - opt_param.sync_time_threshold]
            self.twist_data = self.twist_data.reset_index(drop=True)
        else:
            self.ref_data = self.ref_data[self.ref_data["time"] > self.twist_data["time"][0] - opt_param.sync_time_threshold]
            self.ref_data = self.ref_data.reset_index(drop=True)

        if len(self.twist_data) == 0 or len(self.ref_data) == 0:
            raise RuntimeError("Target time and ref time ranges do not match.")

        if self.ref_data.iloc[-1]["time"] < self.twist_data.iloc[-1]["time"]:
            self.twist_data = self.twist_data[self.twist_data["time"] < self.ref_data.iloc[-1]["time"] + opt_param.sync_time_threshold]
            self.twist_data = self.twist_data.reset_index(drop=True)
        else:
            self.ref_data = self.ref_data[self.ref_data["time"] < self.twist_data.iloc[-1]["time"] + opt_param.sync_time_threshold]
            self.ref_data = self.ref_data.reset_index(drop=True)

        if len(self.twist_data) == 0 or len(self.ref_data) == 0:
            raise RuntimeError("Target time and ref time ranges do not match.")

        sync_ref_time_tmp = self.ref_data["time"]

        num = 0
        len_drop_num = 0
        for i in range(len(self.twist_data)):
            if i == 0: continue
            time_tmp = self.twist_data.iloc[i]["time"] - sync_ref_time_tmp + opt_param.leap_time
            sync_index[i] = np.argmin(abs(time_tmp))
            sync_time_tmp = time_tmp[sync_index[i]]
            if sync_time_tmp < opt_param.sync_time_threshold:
                tmp_num = int(sync_index[i])
                set_target_index.append(i)
                num = num + len_drop_num
                drop_num = list(range(0,tmp_num - len_drop_num - 1, 1))
                if not drop_num == None:
                    sync_ref_time_tmp = sync_ref_time_tmp.drop(drop_num, axis=0)
                    len_drop_num = len(drop_num)
                    sync_ref_time_tmp = sync_ref_time_tmp.reset_index(drop=True)
                set_ref_index.append(num)

        self.ref_data = self.ref_data.iloc[set_ref_index].reset_index(drop=True)
        self.twist_data = self.twist_data.iloc[set_target_index].reset_index(drop=True)

    def get_dr_twist(self, opt_param: OptParam) -> Tuple[pd.DataFrame, pd.DataFrame]:
        last_distance = 0
        last_heading_integtation = 0
        set_dr_trajcetory = []
        set_calc_error = []
        for i in range(len(self.ref_data["time"])):
            if i == 0: continue
            if last_distance + opt_param.distance_step < self.ref_data["distance"][i]:
                data_set_flag = True
                start_distance = self.ref_data["distance"][i]
                start_pos_x = self.ref_data["x"][i]
                start_pos_y = self.ref_data["y"][i]
                previous_pos_x = 0
                previous_pos_y = 0
                last_distance = start_distance
                last_time = self.ref_data["time"][i]
                for j in range(i, len(self.ref_data["time"])):
                    if data_set_flag == True:
                        last_time = self.ref_data["time"][j]
                        last_heading_integtation = np.deg2rad(self.ref_data["yaw"])[j]
                        distance_data = self.ref_data["distance"][j] - start_distance
                        absolute_pos_x = self.ref_data["x"][j] - start_pos_x
                        absolute_pos_y = self.ref_data["y"][j] - start_pos_y
                        data_set_flag = False
                        continue
                    else:
                        heading_integtation = last_heading_integtation + self.twist_data["angular_z"][j] * (self.ref_data["time"][j] - last_time)
                        if opt_param.based_heading_angle == True:
                            usr_vel_x = math.sin(heading_integtation) * self.twist_data["linear_x"][j]
                            usr_vel_y = math.cos(heading_integtation) * self.twist_data["linear_x"][j]
                        else:
                            usr_vel_x = math.cos(heading_integtation) * self.twist_data["linear_x"][j]
                            usr_vel_y = math.sin(heading_integtation) * self.twist_data["linear_x"][j]
                    if self.ref_data["distance"][j] - start_distance < opt_param.distance_length:
                        dr_pos_x = previous_pos_x + usr_vel_x * (self.ref_data["time"][j] - last_time)
                        dr_pos_y = previous_pos_y + usr_vel_y * (self.ref_data["time"][j] - last_time)
                        last_heading_integtation = heading_integtation
                        previous_pos_x = dr_pos_x
                        previous_pos_y = dr_pos_y
                        last_time = self.ref_data["time"][j]
                        distance_data = self.ref_data["distance"][j] - start_distance
                        absolute_pos_x = self.ref_data["x"][j] - start_pos_x
                        absolute_pos_y = self.ref_data["y"][j] - start_pos_y
                        absolute_dr_pos_x = start_pos_x + dr_pos_x
                        absolute_dr_pos_y = start_pos_y + dr_pos_y
                        set_dr_trajcetory.append([absolute_dr_pos_x, absolute_dr_pos_y])
                    else:
                        error_x = absolute_pos_x - dr_pos_x
                        error_y = absolute_pos_y - dr_pos_y
                        error_2d_fabs = math.fabs(error_x ** 2 + error_y ** 2)
                        error_2d = math.sqrt(error_2d_fabs)
                        set_calc_error.append([start_distance, distance_data, absolute_pos_x, dr_pos_x, absolute_pos_y, dr_pos_y, error_x, error_y, error_2d])
                        break
        dr_error = pd.DataFrame(set_calc_error, columns=[
            "start_distance", "distance",
            "absolute_pos_x", "absolute_pos_y",
            "dr_pos_x", "dr_pos_y",
            "error_x", "error_y", "error_2d"
        ])
        dr_trajcetory = pd.DataFrame(set_dr_trajcetory, columns=["x", "y"])
        return dr_error, dr_trajcetory

    def get_dr_rate(self, opt_param: OptParam) -> pd.DataFrame:
        ErrTra_cnt = []
        set_ErrTra_middle = []
        set_ErrTra = []
        ErrTra_cnt = len(self.dr_error["error_2d"])

        step = 0.01
        eval_step_middle = 0.1
        for i in np.arange(0, eval_step_middle, step):
            cnt = 0
            eval_step = i
            for data in self.dr_error["error_2d"]:
                if data < eval_step:
                    cnt = cnt + 1
            rate = cnt / ErrTra_cnt
            set_ErrTra_middle.append([eval_step,rate,rate *100])
        ErrTra_Rate_middle = pd.DataFrame(set_ErrTra_middle,columns=["x_label", "ErrTra_Rate", "ErrTra"])

        step = 0.1
        for i in np.arange(eval_step_middle, opt_param.eval_step_max, step):
            cnt = 0
            eval_step = i
            for data in self.dr_error["error_2d"]:
                if data < eval_step:
                    cnt = cnt + 1
            rate = cnt / ErrTra_cnt
            set_ErrTra.append([eval_step,rate,rate *100])
        ErrTra_Rate_tmp = pd.DataFrame(set_ErrTra,columns=["x_label", "ErrTra_Rate", "ErrTra"])
        ErrTra_Rate = pd.concat([ErrTra_Rate_middle,ErrTra_Rate_tmp])
        return ErrTra_Rate

    @staticmethod
    def calc_distance(df: pd.DataFrame) -> dict:
        if all(key in df.columns for key in ["vel_x", "vel_y", "vel_z"]):
            elapsed = df["time"] - df["time"][0]
            velocity = (df["vel_x"] ** 2 + df["vel_y"] ** 2 + df["vel_z"] ** 2) ** 0.5
            displacement = elapsed * velocity
        else:
            displacement = [0] + [np.sqrt(pow(dx, 2) + pow(dy, 2)) for dx, dy in zip(
                df["x"].iloc[1:].values - df["x"].iloc[:-1].values,
                df["y"].iloc[1:].values - df["y"].iloc[:-1].values,
            )]
        return {"distance": [sum(displacement[:idx + 1]) for idx in range(len(displacement))]}

    @staticmethod
    def calc_elapsed(df: pd.DataFrame) -> dict:
        return {"elapsed": df["time"] - df["time"][0]}

    @staticmethod
    def read_ref_data(param: DataParam) -> pd.DataFrame:
        original_data = pd.read_csv(param.path)
        adjusted_data = pd.DataFrame()

        # time
        if param.separate_time_stamp:
            adjusted_data["time"] = original_data.iloc[:, param.secs_stamp_column] + original_data.iloc[:, param.nsecs_stamp_column] / 10 ** 9
        elif len(str(int(original_data.iloc[0, param.stamp_column]))) > 10:
            adjusted_data["time"] = original_data.iloc[:, param.stamp_column] / 10 ** 9
        else:
            adjusted_data["time"] = original_data.iloc[:, param.stamp_column]

        # position
        adjusted_data["x"] = original_data.iloc[:, param.x_column]
        adjusted_data["y"] = original_data.iloc[:, param.y_column]
        adjusted_data["z"] = original_data.iloc[:, param.z_column]

        # rotation
        if param.use_quaternion:
            for i in range(len(original_data)):
                ref_q_temp = [
                    original_data.iloc[i, param.ori_x_column],
                    original_data.iloc[i, param.ori_y_column],
                    original_data.iloc[i, param.ori_z_column],
                    original_data.iloc[i, param.ori_w_column],
                ]
                ref_e_temp = Rotation.from_quat([ref_q_temp[0], ref_q_temp[1], ref_q_temp[2], ref_q_temp[3]])
                adjusted_data.at[i, "roll"] = ref_e_temp.as_euler("ZYX", degrees=True)[2]
                adjusted_data.at[i, "pitch"] = ref_e_temp.as_euler("ZYX", degrees=True)[1]
                adjusted_data.at[i, "yaw"] = ref_e_temp.as_euler("ZYX", degrees=True)[0]
        elif (not param.use_quaternion) and param.use_radian:
            adjusted_data["roll"] = original_data.iloc[:, param.roll_column] * 180 / math.pi
            adjusted_data["pitch"] = original_data.iloc[:, param.pitch_column] * 180 / math.pi
            adjusted_data["yaw"] = original_data.iloc[:, param.yaw_column] * 180 / math.pi
        elif (not param.use_quaternion) and (not param.use_radian):
            adjusted_data["roll"] = original_data.iloc[:, param.roll_column] * math.pi / 180
            adjusted_data["pitch"] = original_data.iloc[:, param.pitch_column] * math.pi / 180
            adjusted_data["yaw"] = original_data.iloc[:, param.yaw_column] * math.pi / 180

        # enu velocity
        if param.use_enu_velocity:
            adjusted_data["vel_x"] = original_data.iloc[:, param.vel_x_column]
            adjusted_data["vel_y"] = original_data.iloc[:, param.vel_y_column]
            adjusted_data["vel_z"] = original_data.iloc[:, param.vel_z_column]
        
        # angular
        if param.use_angular:
            adjusted_data["angular_x"] = original_data.iloc[:, param.angular_x_column]
            adjusted_data["angular_y"] = original_data.iloc[:, param.angular_y_column]
            adjusted_data["angular_z"] = original_data.iloc[:, param.angular_z_column]

        # GNSS quality
        if param.use_gnss_quality:
            adjusted_data["gnss_quality"] = original_data.iloc[:, param.gnss_quality]

        return adjusted_data

    @staticmethod
    def read_twist_data(param: DataParam) -> pd.DataFrame:
        original_data = pd.read_csv(param.path)
        adjusted_data = pd.DataFrame()

        # time
        if param.separate_time_stamp:
            adjusted_data["time"] = original_data.iloc[:, param.secs_stamp_column] + original_data.iloc[:, param.nsecs_stamp_column] / 10 ** 9
        elif len(str(int(original_data.iloc[0, param.stamp_column]))) > 10:
            adjusted_data["time"] = original_data.iloc[:, param.stamp_column] / 10 ** 9
        else:
            adjusted_data["time"] = original_data.iloc[:, param.stamp_column]
        
        # linear
        adjusted_data["linear_x"] = original_data.iloc[:, param.linear_x_column] # velocity
        adjusted_data["linear_y"] = original_data.iloc[:, param.linear_y_column]
        adjusted_data["linear_z"] = original_data.iloc[:, param.linear_z_column]

        # angluar
        adjusted_data["angular_x"] = original_data.iloc[:, param.angular_x_column]
        adjusted_data["angular_y"] = original_data.iloc[:, param.angular_y_column]
        adjusted_data["angular_z"] = original_data.iloc[:, param.angular_z_column]

        return adjusted_data

def param2pack(ref_param: DataParam, twist_param: DataParam, opt_param: OptParam) -> DataPack:
    return DataPack(ref_param, twist_param, opt_param)
