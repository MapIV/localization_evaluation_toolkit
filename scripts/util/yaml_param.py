import pandas as pd
import numpy as np
import copy

class YamlParam:
    def __init__(self):
        self.separate_time_stamp = ""
        self.stamp_column = ""
        self.secs_stamp_column = ""
        self.nsecs_stamp_column = ""
        self.x_column = ""
        self.y_column = ""
        self.z_column = ""
        self.use_quaternion = ""
        self.ori_x_column = ""
        self.ori_y_column = ""
        self.ori_z_column = ""
        self.ori_w_column = ""
        self.use_radian = ""
        self.roll_column = ""
        self.pitch_column = ""
        self.yaw_column = ""

        self.use_radian = ""
        # self.font = ''
        self.title_font_size = ""
        self.label_font_size = ""
        self.ticks_font_size = ""
        self.save_figures = ""
        self.save_extension_type = ""
        self.save_dataframe = ""

        self.direct_vel = ""
        self.vel_colmn = ""

        self.df_temp = pd.DataFrame()
        self.df = pd.DataFrame()

# ##YamlParamが使えない
# class UnitExtraction(YamlParam):
    def extract_time(self):
        if self.separate_time_stamp == True:
            self.df['time'] = self.df_temp.iloc[:, self.secs_stamp_column] + self.df_temp.iloc[:, self.nsecs_stamp_column] / 10**9
        else:
            self.df['time'] = self.df_temp.iloc[:, self.stamp_column]


    def extract_vel(self):
        if self.separate_time_stamp == True:
            self.df['vel'] = self.df_temp.iloc[:, self.vel_colmn]
        else:
            for i in range(self.df):
                if i==0:
                    continue
                dis_x = self.df_temp.iloc[i, self.x_column] - self.df_temp.iloc[i-1, self.x_column]
                dis_y = self.df_temp.iloc[i, self.y_column] - self.df_temp.iloc[i-1, self.y_column]
                dis_z = self.df_temp.iloc[i, self.z_column] - self.df_temp.iloc[i-1, self.z_column]
                del_time = self.df.loc[i, 'time'] - self.df.loc[i-1, 'time']
                self.df.at[i,'vel'] = np.sqrt(pow(dis_x,2) + pow(dis_y,2) + pow(dis_z,2))/ del_time # [m/s]