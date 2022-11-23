import sys, argparse

import pandas as pd
import yaml

sys.dont_write_bytecode = True

from util import adjust, plot, yaml_param

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='localization evaluation toolkit')
    parser.add_argument('file_paths', nargs="+")
    parser.add_argument('-c', '--config', required=True)
    parser.add_argument('-o', '--outdir', required=True)

    args = parser.parse_args()
    if len(args.file_paths) < 2:
        print("[ERROR]: No enough parameters for reference and results")
        sys.exit(1)

    ref_csv_path = args.file_paths[0]
    result_csv_paths = args.file_paths[1:]
    config_path = args.config
    output_dir = args.outdir

    print("Loading yaml file ...", end="")
    with open(config_path, "r") as yml:
        config = yaml.safe_load(yml)
    ref_param = yaml_param.YamlParam()
    result_params = list()
    save_param = yaml_param.SaveParam()
    op_param = yaml_param.OpParam()
    adjust.input_yaml_param(config, ref_param, "Reference")
    adjust.set_tf(config, ref_param, "Reference")
    for key in config:
        if not key.startswith("Result"): continue
        param = yaml_param.YamlParam()
        adjust.input_yaml_param(config, param, key)
        adjust.set_tf(config, param, key)
        result_params.append(param)
    adjust.input_save_param(config, save_param)
    adjust.input_op_param(config, op_param)
    print("Completed!!")

    print("Loading csv files ...", end="")
    ref_df_org = pd.read_csv(ref_csv_path)
    result_df_orgs = [pd.read_csv(path) for path in result_csv_paths]
    print("Completed!!")

    print("Adjusting unit ...", end="")
    adjust.unit_adjust(ref_param, ref_df_org)
    for result_param, result_df_org in zip(result_params, result_df_orgs):
        adjust.unit_adjust(result_param, result_df_org)
    print("Completed!!")

    if op_param.display_ellipse == True:
        print("Add covariance ...", end="")
        for result_param, result_df_org in zip(result_params, result_df_orgs):
            adjust.add_covariance(result_param, result_df_org, op_param)
        print("Completed!!")
    del ref_df_org, result_df_orgs

    print("Adjusting the start time ...", end="")
    for result_param in sorted(result_params, key=lambda x: x.df_temp.at[0, "time"], reverse=True): # from later to earlier
        if adjust.adjust_start_time(ref_param, result_param) == -1:
            sys.exit(1)
    print("Completed!!")

    print("Adjusting the end time ...", end="")
    for result_param in sorted(result_params, key=lambda x: x.df_temp["time"].iloc[-1]): # from earlier to later
        adjust.adjust_end_time(ref_param, result_param)
    print("Completed!!")

    print("Synchronizing time...", end="")
    for result_param in sorted(result_params, key=lambda x: len(x.df_temp.index)): # from smaller to larger
        ref_param.df, result_param.df = adjust.sync_time(ref_param, result_param, op_param)
        ref_param.df_temp = ref_param.df
        del result_param.df_temp
    del ref_param.df_temp
    print("Completed!!")

    if op_param.display_ellipse == True:
        print("Calculating error ellipse ...", end="")
        for result_param in result_params:
            adjust.calc_ellipse(result_param)
        print("Completed!!")

    print("Output graph ...", end="")
    plot.output_graph(ref_param, result_params, output_dir, save_param, op_param)
