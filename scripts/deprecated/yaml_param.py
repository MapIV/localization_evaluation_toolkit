import pandas as pd


class YamlParam:
    def __init__(self):
        # Auxiliary
        self.label = ""
        self.path = ""

        # CSV
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
        self.tf_time = ""
        self.tf_x = ""
        self.tf_y = ""
        self.tf_z = ""
        self.tf_roll = ""
        self.tf_pitch = ""
        self.tf_yaw = ""
        self.inv_roll = ""
        self.inv_pitch = ""
        self.inv_yaw = ""

        # Bag
        self.topic = ""
        self.bag_id = ""
        self.bag_format = ""

        # Adjust time
        self.direct_vel = ""
        self.vel_colmn = ""

        # DataFrame
        self.df_temp = pd.DataFrame()
        self.df = pd.DataFrame()


class SaveParam:
    def __init__(self):
        # Save format
        self.axis_type = ""
        self.dilution_step = ""
        self.display_radian = ""
        self.progress_info = ""
        self.interval = ""
        self.title_font_size = ""
        self.label_font_size = ""
        self.ticks_font_size = ""
        self.save_figures = ""
        self.save_extension_type = ""
        self.save_dataframe = ""
        self.output_directory = ""


class OpParam:
    def __init__(self):
        self.use_lerp = ""
        self.display_ellipse = ""
        self.covariance_xx_column = ""
        self.covariance_xy_column = ""
        self.covariance_yx_column = ""
        self.covariance_yy_column = ""
