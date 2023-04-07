# configuration reader

from typing import Tuple

class DataParam:
    def __init__(self, config: dict, key: str) -> None:
        # auxiliary info
        self.label = config[key]["label"]
        self.path = config[key]["path"]
        # time
        self.separate_time_stamp = config[key]["separate_time_stamp"]
        self.stamp_column = config[key]["stamp_column"]
        self.secs_stamp_column = config[key]["secs_stamp_column"]
        self.nsecs_stamp_column = config[key]["nsecs_stamp_column"]
        # other entries
        if key == "Reference":
            self.init_ref_param(config, key)
        elif key == "Twist":
            self.init_twist_param(config, key)
        else:
            raise RuntimeError(f"Unexpected config key: {key}")
    
    def init_ref_param(self, config: dict, key: str) -> None:
        try:
            # position
            self.x_column = config[key]["x_column"]
            self.y_column = config[key]["y_column"]
            self.z_column = config[key]["z_column"]
            # rotation
            self.use_quaternion = config[key]["use_quaternion"]
            self.ori_x_column = config[key]["ori_x_column"]
            self.ori_y_column = config[key]["ori_y_column"]
            self.ori_z_column = config[key]["ori_z_column"]
            self.ori_w_column = config[key]["ori_w_column"]
            self.use_radian = config[key]["use_radian"]
            self.roll_column = config[key]["roll_column"]
            self.pitch_column = config[key]["pitch_column"]
            self.yaw_column = config[key]["yaw_column"]
            # enu velocity
            self.use_enu_velocity = config[key]["use_enu_vel"]
            self.vel_x_column = config[key]["vel_x_column"]
            self.vel_y_column = config[key]["vel_y_column"]
            self.vel_z_column = config[key]["vel_z_column"]
            # angular
            self.use_angular = config[key]["use_angular"]
            self.angular_x_column = config[key]["angular_x_column"]
            self.angular_y_column = config[key]["angular_y_column"]
            self.angular_z_column = config[key]["angular_z_column"]
            # GNSS quality
            self.use_gnss_quality = config[key]["use_gnss_qual"]
            self.gnss_quality = config[key]["gnss_qual"]
        except KeyError:
            raise RuntimeError("Undefined config key for the reference data")

    def init_twist_param(self, config: dict, key: str) -> None:
        try:
            # linear
            self.linear_x_column = config[key]["linear_x_column"]
            self.linear_y_column = config[key]["linear_y_column"]
            self.linear_z_column = config[key]["linear_z_column"]
            # angular
            self.angular_x_column = config[key]["angular_x_column"]
            self.angular_y_column = config[key]["angular_y_column"]
            self.angular_z_column = config[key]["angular_z_column"]
        except KeyError:
            raise RuntimeError("Undefined config key for the twist data")

class OptParam:
    def __init__(self, config: dict) -> None:
        # trajectory
        self.progress_info = config["progress_info"]
        self.interval = config["interval"]
        # misc
        self.sync_time_threshold = config["sync_time_threshold"]
        self.leap_time = config["leap_time"]
        self.based_heading_angle = config["based_heading_angle"]
        self.distance_length = config["distance_length"]
        self.distance_step = config["distance_step"]
        self.eval_step_max = config["eval_step_max"]
        # font size
        self.title_font_size = config["title_font_size"]
        self.label_font_size = config["label_font_size"]
        self.ticks_font_size = config["ticks_font_size"]
        # saving option
        self.save_figures = config["save_figures"]
        self.save_extension_type = config["save_extension_type"]
        self.save_dataframe = config["save_dataframe"]
        self.output_directory = config["output_directory"]

def yaml2params(config: dict) -> Tuple[DataParam, DataParam, OptParam]:
    ref_param = DataParam(config, "Reference")
    twist_param = DataParam(config, "Twist")
    opt_param = OptParam(config)
    return ref_param, twist_param, opt_param
