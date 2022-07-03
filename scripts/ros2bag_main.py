import pandas as pd
import sys

sys.dont_write_bytecode = True

from util import yaml_param
from util import adjust
from util import plot
from util import read_ros2bag

if __name__ == "__main__":

    argv = sys.argv
    ref_path = argv[1]
    result_path = argv[2]
    config_path = argv[3]
    output_dir = argv[4]

    print("Loading yaml file ...", end="")
    ref_param = yaml_param.YamlParam()
    result_param = yaml_param.YamlParam()
    save_param = yaml_param.YamlParam()
    config = adjust.input_yaml_ros2(config_path, ref_param, result_param)
    adjust.input_save_param(config, save_param)
    print("Completed!!")

    print("Loading ros2 bag files ...", end="")
    read_ros2bag.read_ros2bag(ref_path, ref_param)
    read_ros2bag.read_ros2bag(result_path, result_param)
    print("Completed!!")

    print("Adjusting the start time ...", end="")
    if adjust.adjust_start_time(ref_param, result_param) == -1:
        sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    adjust.adjust_end_time(ref_param, result_param)
    print("Completed!!")

    print("Synchronizing time...", end="")
    ref_param.df, result_param.df = adjust.sync_time(ref_param, result_param)
    del ref_param.df_temp, result_param.df_temp
    print("Completed!!")

    print("Output graph ...", end="")
    plot.output_graph(ref_param, result_param, output_dir, save_param)
