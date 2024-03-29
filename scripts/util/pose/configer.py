# configuration reader

from typing import List, Tuple

class DataParam:
    def __init__(self, config: dict, key: str) -> None:
        # auxiliary info
        self.label = config[key]["label"]
        self.type = config[key]["type"]
        self.path = config[key]["path"]
        # tf
        self.tf_time = config[key]["tf_time"]
        self.tf_x = config[key]["tf_x"]
        self.tf_y = config[key]["tf_y"]
        self.tf_z = config[key]["tf_z"]
        self.tf_roll = config[key]["tf_roll"]
        self.tf_pitch = config[key]["tf_pitch"]
        self.tf_yaw = config[key]["tf_yaw"]
        self.inv_roll = config[key]["inv_roll"]
        self.inv_pitch = config[key]["inv_pitch"]
        self.inv_yaw = config[key]["inv_yaw"]
        # ellipse
        self.display_ellipse = config[key].get("display_ellipse", False)
        # other entries
        if self.type == 0:
            self.init_csv_param(config, key)
        elif self.type == 1:
            self.init_ros2bag_param(config, key)
        else:
            raise RuntimeError("Unexpected data type")

    def init_csv_param(self, config: dict, key: str) -> None:
        try:
            # time
            self.separate_time_stamp = config[key]["separate_time_stamp"]
            self.stamp_column = config[key]["stamp_column"]
            self.secs_stamp_column = config[key]["secs_stamp_column"]
            self.nsecs_stamp_column = config[key]["nsecs_stamp_column"]
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
            # ellipse
            if self.display_ellipse:
                self.covariance_xx_column = config[key]["covariance_xx_column"]
                self.covariance_xy_column = config[key]["covariance_xy_column"]
                self.covariance_yx_column = config[key]["covariance_yx_column"]
                self.covariance_yy_column = config[key]["covariance_yy_column"]
        except KeyError:
            raise RuntimeError("Undefined config key for the CSV data type")

    def init_ros2bag_param(self, config: dict, key: str) -> None:
        try:
            # ros bag
            self.topic = config[key]["topic_name"]
            self.bag_id = config[key]["storage_id"]
            self.bag_format = config[key]["serialization_format"]
        except KeyError:
            raise RuntimeError("Undefined config key for the ros2 bag data type")

class OptParam:
    def __init__(self, config: dict) -> None:
        # graph horizontal axis
        self.axis_type = config["axis_type"]
        self.degree_type = config["degree_type"]
        # trajectory
        self.dilution_step = config["dilution_step"]
        self.progress_info = config["progress_info"]
        self.interval = config["interval"]
        # font size
        self.title_font_size = config["title_font_size"]
        self.label_font_size = config["label_font_size"]
        self.ticks_font_size = config["ticks_font_size"]
        # saving option
        self.save_figures = config["save_figures"]
        self.save_extension_type = config["save_extension_type"]
        self.save_dataframe = config["save_dataframe"]
        self.output_directory = config["output_directory"]
        # lerp
        self.use_lerp = config["use_lerp"]

def yaml2params(config: dict) -> Tuple[DataParam, List[DataParam], OptParam]:
    ref_param = DataParam(config, "Reference")
    res_params = [DataParam(config, key) for key in config if key.startswith("Result")]
    opt_param = OptParam(config)
    return ref_param, res_params, opt_param
