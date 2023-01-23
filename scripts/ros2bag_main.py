import sys

import pandas as pd
import yaml

sys.dont_write_bytecode = True

from util import read_ros2bag
from deprecated import adjust, plot, yaml_param

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
    result_params = list()
    result_param = yaml_param.YamlParam()
    result_params.append(result_param)
    save_param = yaml_param.SaveParam()
    op_param = yaml_param.OpParam()
    adjust.input_yaml_ros2(config, ref_param, "Reference")
    adjust.set_tf(config, ref_param, "Reference")
    adjust.input_yaml_ros2(config, result_param, "Result")
    adjust.set_tf(config, result_param, "Result")
    adjust.input_save_param(config, save_param)
    adjust.input_op_param_ros2(config, op_param)
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

    if op_param.display_ellipse == True:
        print("Calculating error ellipse ...", end="")
        adjust.calc_ellipse(result_param)
        print("Completed!!")

    print("Output graph ...", end="")
    plot.output_graph(ref_param, result_params, save_param, op_param)
