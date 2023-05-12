# graph plotter

import matplotlib.pyplot as plt
import matplotlib.ticker
from matplotlib.figure import Figure
from functools import reduce
from typing import Dict

from util.twist.configer import OptParam
from util.twist.packer import DataPack

def plot_relative_position_error(data_pack: DataPack, opt_param: OptParam) -> Dict[str, Figure]:
    fig_rel_pos_err = plt.figure("Relative_Position_Error", figsize=(16, 9), dpi=120)
    ax_rel_pos_err = fig_rel_pos_err.add_subplot(111)

    ax_rel_pos_err.plot(data_pack.dr_error["start_distance"], data_pack.dr_error["error_2d"], markersize=1)
    ax_rel_pos_err.set_title("Relative Position Error", fontsize=opt_param.title_font_size)
    ax_rel_pos_err.set_xlabel("start distance [m]", fontsize=opt_param.label_font_size)
    ax_rel_pos_err.set_ylabel("2D error [m]", fontsize=opt_param.label_font_size)
    ax_rel_pos_err.tick_params(labelsize=opt_param.ticks_font_size)
    ax_rel_pos_err.grid()

    return {"relative_position_error": fig_rel_pos_err}

def plot_cumulative_error_distribution(data_pack: DataPack, opt_param: OptParam) -> Dict[str, Figure]:
    fig_cum_err_dis = plt.figure("Cumulative_Error_Distribution", figsize=(16, 9), dpi=120)
    ax_cum_err_dis = fig_cum_err_dis.add_subplot(111)

    ax_cum_err_dis.plot(data_pack.dr_rate["x_label"], data_pack.dr_rate["ErrTra"], marker="s", markersize=10)
    ax_cum_err_dis.set_title("Cumulative Error Distribution (relative position)", fontsize=opt_param.title_font_size)
    ax_cum_err_dis.set_xlabel("2D error [m]", fontsize=opt_param.label_font_size)
    ax_cum_err_dis.set_xscale("log")
    ax_cum_err_dis.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_cum_err_dis.set_xticks([0.01, 0.05, 0.1, 0.5, 1.0, 3.0])
    ax_cum_err_dis.set_ylabel("Rate [%]", fontsize=opt_param.label_font_size)
    ax_cum_err_dis.tick_params(labelsize=opt_param.ticks_font_size)
    ax_cum_err_dis.grid()

    return {"cumulative_error_distribution": fig_cum_err_dis}

def plot_dr_trajectory(data_pack: DataPack, opt_param: OptParam) -> Dict[str, Figure]:
    fig_dr_traj = plt.figure("DR_Trajectory", figsize=(16, 9), dpi=120)
    ax_dr_traj = fig_dr_traj.add_subplot(111)

    ax_dr_traj.plot(
        data_pack.ref_data["x"] - data_pack.ref_data["x"][0], data_pack.ref_data["y"] - data_pack.ref_data["y"][0],
        marker=".", markersize=3, linestyle="None", label=data_pack.ref_label
    )
    ax_dr_traj.plot(
        data_pack.dr_traj["x"] - data_pack.ref_data["x"][0], data_pack.dr_traj["y"] - data_pack.ref_data["y"][0],
        marker="s", markersize=3, linestyle="None", alpha=0.3, label=data_pack.twist_label
    )
    # show progress
    if opt_param.progress_info:
        if opt_param.progress_info == 1:
            data = data_pack.ref_data.index
        elif opt_param.progress_info == 2:
            data = data_pack.ref_data["elapsed"]
        elif opt_param.progress_info == 3:
            data = data_pack.ref_data["time"]
        elif opt_param.progress_info == 4:
            data = data_pack.ref_data["distance"]
        else:
            raise RuntimeError("Unexpected progress info type")
        counter = 0
        for i in data_pack.ref_data.index:
            if (data[i] - data[0]) < counter: continue
            ax_dr_traj.text(
                data_pack.ref_data["x"][i] - data_pack.ref_data["x"][0],
                data_pack.ref_data["y"][i] - data_pack.ref_data["y"][0],
                round(data[i], 2), va="bottom"
            )
            counter += opt_param.interval
    ax_dr_traj.set_title("DR Trajectory", fontsize=opt_param.title_font_size)
    ax_dr_traj.set_xlabel("East [m]", fontsize=opt_param.label_font_size)
    ax_dr_traj.set_ylabel("West [m]", fontsize=opt_param.label_font_size)
    ax_dr_traj.legend()
    ax_dr_traj.tick_params(labelsize=opt_param.ticks_font_size)
    ax_dr_traj.grid()
    ax_dr_traj.set_aspect("equal")
    ax_dr_traj.axis("square")

    return {"dr_trajectory": fig_dr_traj}

def plot(data_pack: DataPack, opt_param: OptParam) -> Dict[str, Figure]:
    figs = reduce(lambda x, y: {**x, **y}, [
        plot_relative_position_error(data_pack, opt_param),
        plot_cumulative_error_distribution(data_pack, opt_param),
        plot_dr_trajectory(data_pack, opt_param),
    ])
    plt.show()
    return figs

def save_df(data_pack: DataPack, opt_param: OptParam) -> None:
    cols = ["start_distance", "error_2d"]
    data_pack.dr_error[cols].to_csv(f"{opt_param.output_directory}/target_dr_error.csv", header=True, index=False, float_format="%.9f")

def save_figures(figs: Dict[str, Figure], opt_param: OptParam) -> None:
    for name, fig in figs.items():
        fig.savefig(f"{opt_param.output_directory}/{name}.{opt_param.save_extension_type}")

def save(data_pack: DataPack, figs: Dict[str, Figure], opt_param: OptParam) -> None:
    # save dataframes
    if opt_param.save_dataframe:
        save_df(data_pack, opt_param)
    # save figures
    if opt_param.save_figures:
        save_figures(figs, opt_param)
