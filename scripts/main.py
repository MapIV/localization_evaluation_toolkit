import sys

import pandas as pd
import yaml

sys.dont_write_bytecode = True

from util import adjust, plot, yaml_param

if __name__ == "__main__":

    argv = sys.argv
    ref_csv_path = argv[1]
    result_csv_path = argv[2]
    config_path = argv[3]
    output_dir = argv[4]

    print("Loading yaml file ...", end="")
    with open(config_path, "r") as yml:
        config = yaml.safe_load(yml)
    ref_param = yaml_param.YamlParam()
    result_param = yaml_param.YamlParam()
    save_param = yaml_param.SaveParam()
    op_param = yaml_param.OpParam()
    adjust.input_yaml_param(config, ref_param, "Reference")
    adjust.set_tf(config, ref_param, "Reference")
    adjust.input_yaml_param(config, result_param, "Result")
    adjust.set_tf(config, result_param, "Result")
    adjust.input_save_param(config, save_param)
    adjust.input_op_param(config, op_param)
    print("Completed!!")

    print("Loading csv files ...", end="")
    ref_df_org = pd.read_csv(ref_csv_path)
    result_df_org = pd.read_csv(result_csv_path)
    print("Completed!!")

    print("Adjusting unit ...", end="")
    adjust.unit_adjust(ref_param, ref_df_org)
    adjust.unit_adjust(result_param, result_df_org)
    del ref_df_org, result_df_org
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

    print("Output graph ...", end="")
    plot.output_graph(ref_param, result_param, output_dir, save_param, op_param)
