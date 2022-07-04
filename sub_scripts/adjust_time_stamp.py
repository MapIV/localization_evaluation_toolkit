import numpy as np
import pandas as pd
import sys
import yaml
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
sys.path.append('../scripts')
from util import yaml_param
from util import extract

def input_yaml_param(config_path, target_param, offset_param):
    with open(config_path, "r") as yml:
        config = yaml.safe_load(yml)

        target_param.separate_time_stamp = config["Target"]["separate_time_stamp"]
        target_param.stamp_column = config["Target"]["stamp_column"]
        target_param.secs_stamp_column = config["Target"]["secs_stamp_column"]
        target_param.nsecs_stamp_column = config["Target"]["nsecs_stamp_column"]    
        target_param.direct_vel = config["Target"]["direct_vel"]
        target_param.vel_colmn = config["Target"]["vel_colmn"]        
        target_param.x_column = config["Target"]["x_column"]
        target_param.y_column = config["Target"]["y_column"]
        target_param.z_column = config["Target"]["z_column"]

        offset_param.separate_time_stamp = config["Offset"]["separate_time_stamp"]
        offset_param.stamp_column = config["Offset"]["stamp_column"]
        offset_param.secs_stamp_column = config["Offset"]["secs_stamp_column"]
        offset_param.nsecs_stamp_column = config["Offset"]["nsecs_stamp_column"]    
        offset_param.direct_vel = config["Offset"]["direct_vel"]
        offset_param.vel_colmn = config["Offset"]["vel_colmn"]        
        offset_param.x_column = config["Offset"]["x_column"]
        offset_param.y_column = config["Offset"]["y_column"]
        offset_param.z_column = config["Offset"]["z_column"]

        search_range = config["search_range"]
        delta = config["delta"]

        return (search_range, delta)

def initial_graph(target_param, offset_param):
    initial_fig= plt.figure("Initial Graph", figsize=(16, 9), dpi=120)
    ax_initial = initial_fig.add_subplot(111)
    ax_initial.set_title("Initial Graph")
    ax_initial.scatter(target_param.df['time'], target_param.df['vel'], c="k")
    ax_initial.scatter(offset_param.df['time'], offset_param.df['vel'],  c="r", s=2)
    ax_initial.set_xlabel('time[s]')
    ax_initial.set_ylabel('data')
    ax_initial.grid()
    ax_initial.set_aspect('equal')


def all_search(target_df, offset_df, search_range, delta):
    num_off = search_range / delta
    target = target_df.to_numpy()
    offset_df["time"] -= search_range
    calc_df = offset_df.copy(deep=True)
    dist_sum = []

    # kd_tree
    kd_tree = KDTree(target)
    for i in range(int(num_off)*2):
        offset = calc_df.to_numpy()
        dist_temp, indexes_temp = kd_tree.query(offset)
        dist_sum.append(np.sum(dist_temp))
        calc_df["time"] += delta
    min_idx = dist_sum.index(min(dist_sum))
    offset_df["time"] += delta*min_idx
    fit_width = delta * min_idx - search_range
    return (fit_width)


def output(fit_width, target_param, offset_param, output_dir):

    if offset_param.separate_time_stamp == True:
        offset_param.df_temp.iloc[:,offset_param.secs_stamp_column] = offset_param.df["time"]
        offset_param.df_temp.drop(offset_param.df_temp.columns[offset_param.nsecs_stamp_column], axis=1, inplace=True)
    else:
        offset_param.df_temp.iloc[:,offset_param.stamp_column] = offset_param.df["time"]

    # Output as csv
    offset_param.df_temp.to_csv(output_dir + "/after_time_offset.csv", index = False)

    # Output matched graph
    fit_fig= plt.figure("Matched Graph", figsize=(16, 9), dpi=120)
    ax_fit = fit_fig.add_subplot(111)
    ax_fit.set_title("Matched Graph")
    ax_fit.scatter(target_param.df['time'], target_param.df['vel'], c="k")
    ax_fit.scatter(offset_param.df['time'], offset_param.df['vel'],  c="r", s=2)
    ax_fit.text(0.1,1.01, 'Offset: {} [s]'.format(fit_width), fontsize=15, transform=ax_fit.transAxes)
    ax_fit.set_xlabel('time[s]')
    ax_fit.set_ylabel('data')
    ax_fit.grid()
    ax_fit.set_aspect('equal')
    print("Completed!!")
    plt.show()


if __name__ == "__main__":
    argv = sys.argv
    target_dir = argv[1]
    offset_dir = argv[2]
    config_path = argv[3]
    output_dir = argv[4]

    print("Loading yaml file ...", end="")
    target_param = yaml_param.YamlParam()
    offset_param = yaml_param.YamlParam()
    search_range, delta = input_yaml_param(config_path, target_param, offset_param)
    print("Completed!!")

    print("Loading csv files ...", end="")
    target_param.df_temp = pd.read_csv(target_dir)
    offset_param.df_temp  = pd.read_csv(offset_dir)
    print("Completed!!")

    print("Extraction ...", end="")
    extract.extract_time(target_param)
    extract.extract_vel(target_param)
    extract.extract_time(offset_param)
    extract.extract_vel(offset_param)
    del target_param.df_temp
    initial_graph(target_param, offset_param)
    print("Completed!!")

    print("Find minimum ...", end="")
    fit_width = all_search(target_param.df, offset_param.df, search_range, delta)
    print("Completed!!")

    print("Output data ...", end="")
    output(fit_width, target_param, offset_param, output_dir)
