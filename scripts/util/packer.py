# csv data packer

from __future__ import annotations

import math
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from typing import List, Tuple

from util.configer import CsvParam, OptParam

class DataPack:
    def __init__(self, param: CsvParam, opt_param: OptParam) -> None:
        # set basic info
        self.label: str = param.label
        self.data: pd.DataFrame = None
        # set axis info
        self.axis_name = "elapsed" if opt_param.axis_type == 0 else "distance"
        self.axis_unit = "time[s]" if opt_param.axis_type == 0 else "distance[m]"
        self.degree_formater = (lambda x: x * 1) if opt_param.degree_type == 0 else (lambda x: x * 180 / np.pi)
        self.degree_unit = "[radian]" if opt_param.degree_type == 0 else "[degree]"

    @property
    def axis_array(self) -> pd.Series:
        return self.data[self.axis_name]

    @staticmethod
    def read_data(csv_param: CsvParam, opt_param: OptParam) -> pd.DataFrame:
        original_data = pd.read_csv(csv_param.path)
        adjusted_data = pd.DataFrame()
        
        # time
        if csv_param.separate_time_stamp == True:
            adjusted_data["time"] = original_data.iloc[:, csv_param.secs_stamp_column] + \
                                    original_data.iloc[:, csv_param.nsecs_stamp_column] / 10 ** 9 + csv_param.tf_time
        elif len(str(int(original_data.iloc[0, csv_param.stamp_column]))) > 10:
            adjusted_data["time"] = original_data.iloc[:, csv_param.stamp_column] / 10 ** 9 + csv_param.tf_time
        else:
            adjusted_data["time"] = original_data.iloc[:, csv_param.stamp_column] + csv_param.tf_time
        
        # position
        adjusted_data["x"] = original_data.iloc[:, csv_param.x_column] + csv_param.tf_x
        adjusted_data["y"] = original_data.iloc[:, csv_param.y_column] + csv_param.tf_y
        adjusted_data["z"] = original_data.iloc[:, csv_param.z_column] + csv_param.tf_z

        # rotation
        if csv_param.use_quaternion == True:
            for i in range(len(original_data)):
                ref_q_temp = [
                    original_data.iloc[i, csv_param.ori_x_column],
                    original_data.iloc[i, csv_param.ori_y_column],
                    original_data.iloc[i, csv_param.ori_z_column],
                    original_data.iloc[i, csv_param.ori_w_column],
                ]
                ref_e_temp = Rotation.from_quat([ref_q_temp[0], ref_q_temp[1], ref_q_temp[2], ref_q_temp[3]])
                adjusted_data.at[i, "roll"] = (
                    ref_e_temp.as_euler("ZYX", degrees=False)[2] + csv_param.tf_roll
                ) * csv_param.inv_roll
                adjusted_data.at[i, "pitch"] = (
                    ref_e_temp.as_euler("ZYX", degrees=False)[1] + csv_param.tf_pitch
                ) * csv_param.inv_pitch
                adjusted_data.at[i, "yaw"] = (ref_e_temp.as_euler("ZYX", degrees=False)[0] + csv_param.tf_yaw) * csv_param.inv_yaw
        elif csv_param.use_quaternion == False and csv_param.use_radian == True:
            adjusted_data["roll"] = (original_data.iloc[:, csv_param.roll_column] + csv_param.tf_roll) * csv_param.inv_roll
            adjusted_data["pitch"] = (original_data.iloc[:, csv_param.pitch_column] + csv_param.tf_pitch) * csv_param.inv_pitch
            adjusted_data["yaw"] = (original_data.iloc[:, csv_param.yaw_column] + csv_param.tf_yaw) * csv_param.inv_yaw
        elif csv_param.use_quaternion == False and csv_param.use_radian == False:
            adjusted_data["roll"] = (original_data.iloc[:, csv_param.roll_column] * math.pi / 180 + csv_param.tf_roll) * csv_param.inv_roll
            adjusted_data["pitch"] = (original_data.iloc[:, csv_param.pitch_column] * math.pi / 180 + csv_param.tf_pitch) * csv_param.inv_pitch
            adjusted_data["yaw"] = (original_data.iloc[:, csv_param.yaw_column] * math.pi / 180 + csv_param.tf_yaw) * csv_param.inv_yaw
        
        # covariance
        if opt_param.display_ellipse == True:
            adjusted_data["cov_xx"] = original_data.iloc[:, opt_param.covariance_xx_column]
            adjusted_data["cov_xy"] = original_data.iloc[:, opt_param.covariance_xy_column]
            adjusted_data["cov_yx"] = original_data.iloc[:, opt_param.covariance_yx_column]
            adjusted_data["cov_yy"] = original_data.iloc[:, opt_param.covariance_yy_column]

        return adjusted_data

    @staticmethod
    def calc_elapsed(df: pd.DataFrame) -> dict:
        return {"elapsed": df["time"] - df["time"][0]}
    
    @staticmethod
    def calc_distance(df: pd.DataFrame) -> dict:
        displacement = [0] + [np.sqrt(pow(dx, 2) + pow(dy, 2)) for dx, dy in zip(
            df["x"].iloc[1:].values - df["x"].iloc[:-1].values,
            df["y"].iloc[1:].values - df["y"].iloc[:-1].values,
        )]
        return {"distance": [sum(displacement[:idx + 1]) for idx in range(len(displacement))]}

    @staticmethod
    def calc_velocity(df: pd.DataFrame) -> dict:
        return {"velocity": np.array([np.nan] + [
            np.nan if dt < 0.00001 else np.sqrt(pow(dx, 2) + pow(dy, 2)) / dt
            for dx, dy, dt in zip(
                df["x"].iloc[1:].values - df["x"].iloc[:-1].values,
                df["y"].iloc[1:].values - df["y"].iloc[:-1].values,
                df["time"].iloc[1:].values - df["time"].iloc[:-1].values
            )
        ])}

class ResDataPack(DataPack):
    def __init__(self, res_param: CsvParam, ref_param: CsvParam, opt_param: OptParam) -> None:
        super().__init__(res_param, opt_param)
        # read data
        self.data = self.read_data(res_param, opt_param)
        self.data_ref = self.read_data(ref_param, opt_param)
        # adjust time stamp
        self.adjust_start_time()
        self.adjust_end_time()
        self.sync_time(opt_param)
        # pre-calculation
        self.data_ref = self.data_ref.assign(**self.calc_velocity(self.data_ref))
        self.data = self.data.assign(**self.calc_elapsed(self.data))
        self.data = self.data.assign(**self.calc_distance(self.data))
        # self.data = self.data.assign(**self.calc_velocity(self.data))
        self.data = self.data.assign(**self.calc_error(self.data_ref, self.data))
        if opt_param.display_ellipse:
            self.data = self.data.assign(**self.calc_ellipse(self.data))

    def adjust_start_time(self):
        if (
            self.data_ref.at[0, "time"] < self.data.at[0, "time"] and
            self.data_ref["time"].iloc[-1] > self.data.at[0, "time"]
        ):
            search_ref_start = abs(self.data_ref["time"] - self.data.at[0, "time"])
            ref_start_time_index = search_ref_start.idxmin()
            self.data_ref.drop(range(0, ref_start_time_index), inplace=True)
            self.data_ref.reset_index(inplace=True, drop=True)
        elif (
            self.data_ref.at[0, "time"] > self.data.at[0, "time"] and
            self.data_ref.at[0, "time"] < self.data["time"].iloc[-1]
        ):
            search_result_start = abs(self.data["time"] - self.data_ref.at[0, "time"])
            result_start_time_index = search_result_start.idxmin()
            self.data.drop(range(0, result_start_time_index), inplace=True)
            self.data.reset_index(inplace=True, drop=True)
        elif (
            self.data_ref.at[0, "time"] == self.data.at[0, "time"]
        ):
            pass
        else:
            raise RuntimeError("Reference time stamp and Result time stamp do not overlap")

    def adjust_end_time(self):
        if self.data_ref["time"].iloc[-1] > self.data["time"].iloc[-1]:
            search_ref_end = abs(self.data_ref["time"] - self.data["time"].iloc[-1])
            ref_end_time_index = search_ref_end.idxmin()
            self.data_ref.drop(range(ref_end_time_index + 1, len(self.data_ref.index)), inplace=True)
            self.data_ref.reset_index(inplace=True, drop=True)
        elif self.data_ref["time"].iloc[-1] < self.data["time"].iloc[-1]:
            search_result_end = abs(self.data["time"] - self.data_ref["time"].iloc[-1])
            result_end_time_index = search_result_end.idxmin()
            self.data.drop(range(result_end_time_index + 1, len(self.data.index)), inplace=True)
            self.data.reset_index(inplace=True, drop=True)

    def sync_time(self, opt_param: OptParam):
        before_sync = -1
        before_min_time = 0
        if len(self.data_ref.index) >= len(self.data.index):
            sync_ref_df = pd.DataFrame(columns=self.data_ref.columns)
            for i in range(0, len(self.data)):
                search_sync_ref_time = abs(self.data_ref["time"] - self.data.at[i, "time"])
                sync_ref_id = search_sync_ref_time.idxmin()
                min_ref_time = search_sync_ref_time.min()
                if sync_ref_id == before_sync:
                    if min_ref_time < before_min_time:
                        sync_ref_df.drop(sync_ref_df.index[-1], inplace=True)
                        self.data.drop(i - 1, inplace=True)
                    elif min_ref_time > before_min_time:
                        self.data.drop(i, inplace=True)
                        continue
                if opt_param.use_lerp:
                    self.lerp(self.data_ref, sync_ref_id, min_ref_time, self.data.at[i, "time"])
                sync_ref_df = pd.concat([sync_ref_df, self.data_ref.iloc[[sync_ref_id]]], ignore_index=True)
                before_sync = sync_ref_id
                before_min_time = min_ref_time
            self.data.reset_index(inplace=True, drop=True)
            del search_sync_ref_time, before_min_time
            self.data_ref = sync_ref_df
        else:
            sync_result_df = pd.DataFrame(columns=self.data.columns)
            ref_df_temp = self.data_ref.copy()
            for i in range(0, len(self.data_ref)):
                search_sync_result_time = abs(self.data["time"] - self.data_ref.at[i, "time"])
                sync_result_id = search_sync_result_time.idxmin()
                min_result_time = search_sync_result_time.min()
                if sync_result_id == before_sync:
                    if min_result_time < before_min_time:
                        sync_result_df.drop(sync_result_df.index[-1], inplace=True)
                        self.data_ref.drop(i - 1, inplace=True)
                    elif min_result_time > before_min_time:
                        self.data_ref.drop(i, inplace=True)
                        continue
                if opt_param.use_lerp:
                    self.lerp(ref_df_temp, i, min_result_time, self.data.at[sync_result_id, "time"])
                sync_result_df = pd.concat([sync_result_df, self.data.iloc[[sync_result_id]]], ignore_index=True)
                before_sync = sync_result_id
                before_min_time = min_result_time
            self.data_ref.reset_index(inplace=True, drop=True)
            del search_sync_result_time, before_min_time
            self.data = sync_result_df

    @staticmethod
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

    @staticmethod
    def calc_error(df_ref: pd.DataFrame, df_res: pd.DataFrame) -> dict:
        error_x = df_res["x"] - df_ref["x"]
        error_y = df_res["y"] - df_ref["y"]
        error_z = df_res["z"] - df_ref["z"]
        round2pipi = lambda v: (v % (math.pi * 2)) - (0 if (v % (math.pi * 2)) < math.pi else (math.pi * 2))
        return {
            "error_x": error_x,
            "error_y": error_y,
            "error_z": error_z,
            "error_2d": np.sqrt(pow(error_x, 2) + pow(error_y, 2)),
            "error_3d": np.sqrt(pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2)),
            "error_longitudinal": error_x * np.cos(-df_ref["yaw"]) - error_y * np.sin(-df_ref["yaw"]),
            "error_lateral": error_x * np.sin(-df_ref["yaw"]) + error_y * np.cos(-df_ref["yaw"]),
            "error_roll": (df_res["roll"] - df_ref["roll"]).map(round2pipi),
            "error_pitch": (df_res["pitch"] - df_ref["pitch"]).map(round2pipi),
            "error_yaw": (df_res["yaw"] - df_ref["yaw"]).map(round2pipi),
            # "error_velocity": df_res["velocity"] - df_ref["velocity"],
        }

    @staticmethod
    def calc_ellipse(df: pd.DataFrame) -> dict:
        ellipse_dict = {
            "ellipse_long": [],
            "ellipse_short": [],
            "ellipse_yaw": [],
            "ellipse_longitudinal": [],
            "ellipse_lateral": [],
        }
        for i in range(len(df)):
            covariance = np.array(
                [[df.at[i, "cov_xx"], df.at[i, "cov_xy"]], [df.at[i, "cov_yx"], df.at[i, "cov_yy"]]]
            )
            eig_vals, eig_vec = np.linalg.eig(covariance)
            ellipse_long = math.sqrt(eig_vals[0] * 9.21)
            ellipse_short = math.sqrt(eig_vals[1] * 9.21)
            ellipse_yaw = round(math.atan2(eig_vec[1, 0], eig_vec[0, 0]), 4)
            th = ellipse_yaw - df.at[i, "yaw"]
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
        return ellipse_dict

class RefDataPack(DataPack):
    def __init__(self, ref_param: CsvParam, opt_param: OptParam, df: pd.DataFrame=None) -> None:
        super().__init__(ref_param, opt_param)
        # read data
        self.data = df if isinstance(df, pd.DataFrame) else self.read_data(ref_param, opt_param)
        # pre-calculation
        self.data = self.data.assign(**self.calc_elapsed(self.data))
        self.data = self.data.assign(**self.calc_distance(self.data))
        # self.data = self.data.assign(**self.calc_velocity(self.data))

    @classmethod
    def build_from_res(cls, ref_param: CsvParam, opt_param: OptParam, res_packs: List[ResDataPack]) -> RefDataPack:
        df_len = min(map(len, [res_pack.data_ref for res_pack in res_packs]))
        for res_pack in res_packs:
            selected_rows = np.sort(np.random.choice(len(res_pack.data_ref), df_len, replace=False))
            res_pack.data = res_pack.data.iloc[selected_rows].reset_index(drop=True)
            res_pack.data_ref = res_pack.data_ref.iloc[selected_rows].reset_index(drop=True)
        ref_df = cls.accumulate_df([res_pack.data_ref for res_pack in res_packs])
        selected_rows = np.sort(np.random.choice(len(ref_df), df_len, replace=False))
        return cls(ref_param, opt_param, ref_df.iloc[selected_rows].reset_index(drop=True))

    @staticmethod
    def accumulate_df(dfs: List[pd.DataFrame], sort_by: str="time") -> pd.DataFrame:
        return pd.concat(dfs, ignore_index=True).drop_duplicates().sort_values(by=sort_by, ignore_index=True)

def param2pack(ref_parm: CsvParam, res_params: List[CsvParam], opt_param: OptParam) -> Tuple[RefDataPack, List[ResDataPack]]:
    res_packs = [ResDataPack(res_param, ref_parm, opt_param) for res_param in res_params]
    ref_pack = RefDataPack.build_from_res(ref_parm, opt_param, res_packs)
    return ref_pack, res_packs
