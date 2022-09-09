import numpy as np
import pandas as pd


def extract_time(param):
    if param.separate_time_stamp == True:
        param.df["time"] = (
            param.df_temp.iloc[:, param.secs_stamp_column] + param.df_temp.iloc[:, param.nsecs_stamp_column] / 10**9
        )
    else:
        if len(str(int(param.df_temp.iloc[0, param.stamp_column]))) > 10:
            param.df["time"] = param.df_temp.iloc[:, param.stamp_column] / 10**9
        else:
            param.df["time"] = param.df_temp.iloc[:, param.stamp_column]


def extract_vel(param):
    if param.direct_vel == True:
        param.df["vel"] = param.df_temp.iloc[:, param.vel_colmn]
    else:
        for i in range(len(param.df)):
            if i == 0:
                continue
            dis_x = param.df_temp.iloc[i, param.x_column] - param.df_temp.iloc[i - 1, param.x_column]
            dis_y = param.df_temp.iloc[i, param.y_column] - param.df_temp.iloc[i - 1, param.y_column]
            dis_z = param.df_temp.iloc[i, param.z_column] - param.df_temp.iloc[i - 1, param.z_column]
            del_time = param.df.loc[i, "time"] - param.df.loc[i - 1, "time"]
            param.df.at[i - 1, "vel"] = np.sqrt(pow(dis_x, 2) + pow(dis_y, 2) + pow(dis_z, 2)) / del_time  # [m/s]
        param.df.at[len(param.df) - 1, "vel"] = param.df.at[len(param.df) - 2, "vel"]
