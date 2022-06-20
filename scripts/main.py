import pandas as pd
import sys
sys.dont_write_bytecode = True

from util import yaml_param
from util import adjust
from util import plot

if __name__ == '__main__':

    argv = sys.argv
    ref_csv_dir = argv[1]
    result_csv_dir = argv[2]
    config_dir = argv[3]
    output_dir = argv[4]

    print('Loading yaml file ...', end='')
    ref_param = yaml_param.YamlParam()
    result_param = yaml_param.YamlParam()
    save_param = yaml_param.YamlParam() 
    adjust.input_yaml_param(config_dir, ref_param, result_param, save_param)
    print('Completed!!')

    print('Loading csv files ...', end='')
    ref_df_temp = pd.read_csv(ref_csv_dir)
    result_df_temp = pd.read_csv(result_csv_dir)
    print('Completed!!')

    print('Adjusting unit ...', end='')
    ref_df = pd.DataFrame()
    result_df = pd.DataFrame()
    adjust.unit_adjust(ref_param, result_param, ref_df_temp, result_df_temp, ref_df, result_df)
    del ref_df_temp, result_df_temp
    print('Completed!!')

    print('Adjusting the start time ...', end='')
    if adjust.adjust_start_time(ref_df,result_df) is -1:
        sys.exit(1)
    print('Completed!!')

    print('Adjusting the end time ...', end='')
    adjust.adjust_end_time(ref_df,result_df)
    print('Completed!!')

    print('Synchronizing time...', end='')
    sync_ref_df, sync_result_df = adjust.sync_time(ref_df, result_df)
    del ref_df, result_df
    print('Completed!!')

    print('Output graph ...', end='')
    plot.output_graph(sync_ref_df, sync_result_df, output_dir, save_param)