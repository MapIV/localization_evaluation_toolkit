# graph plotter

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import axes3d, Axes3D
import numpy as np
from functools import reduce
from typing import List, Dict

from util.pose.configer import OptParam
from util.pose.packer import RefDataPack, ResDataPack

def plot_2d_traj(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_2d_trj = plt.figure("2D_Trajectory", figsize=(16, 9), dpi=120)
    ax_2d_trj = fig_2d_trj.add_subplot(111)

    ref_dfs = list()
    for res_pack in res_packs:
        # dilute trajectory data for better performance
        res_df = res_pack.data.iloc[::opt_param.dilution_step]
        ref_df = res_pack.data_ref.iloc[::opt_param.dilution_step]
        ref_dfs.append(ref_df)
        # plot the traj of results
        ax_2d_trj.scatter(res_df["x"], res_df["y"], s=2, label=res_pack.label)
        ax_2d_trj.plot([ref_df["x"], res_df["x"]], [ref_df["y"], res_df["y"]], "-", linewidth=0.2, zorder=1)
        # show ellipse
        if res_pack.display_ellipse:
            for i in res_df.index:
                ax_2d_trj.add_patch(patches.Ellipse(
                    xy=(res_df["x"][i], res_df["y"][i]),
                    angle=np.degrees(res_df["ellipse_yaw"][i]),
                    width=res_df["ellipse_long"][i] * 2,
                    height=res_df["ellipse_short"][i] * 2,
                    alpha=0.3
                ))
        # show progress
        if opt_param.progress_info:
            if opt_param.progress_info == 1:
                data = res_df.index
            elif opt_param.progress_info == 2:
                data = res_df["elapsed"]
            elif opt_param.progress_info == 3:
                data = res_df["time"]
            elif opt_param.progress_info == 4:
                data = res_df["distance"]
            else:
                raise RuntimeError("Unexpected progress info type")
            counter = 0
            for i in res_df.index:
                if (data[i] - data[0]) < counter: continue
                ax_2d_trj.text(res_df["x"][i], res_df["y"][i], round(data[i], 2), va="bottom")
                counter += opt_param.interval
    # plot the traj of reference
    ref_df = ref_pack.accumulate_df(ref_dfs)
    ax_2d_trj.scatter(ref_df["x"], ref_df["y"], c="k", zorder=-1, label=ref_pack.label)

    ax_2d_trj.set_title("2D Trajectory", fontsize=opt_param.title_font_size)
    ax_2d_trj.set_xlabel("x[m]", fontsize=opt_param.label_font_size)
    ax_2d_trj.set_ylabel("y[m]", fontsize=opt_param.label_font_size)
    ax_2d_trj.tick_params(labelsize=opt_param.ticks_font_size)
    ax_2d_trj.legend()
    ax_2d_trj.set_aspect("equal")
    ax_2d_trj.grid()

    return {"2d_trajectory": fig_2d_trj}

def plot_3d_traj(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_3d_trj = plt.figure('3D_Trajectory', figsize=(16, 9), dpi=120)
    ax_3d_trj = fig_3d_trj.add_subplot(projection="3d")

    ax_3d_trj.plot3D(ref_pack.data["x"], ref_pack.data["y"], ref_pack.data["z"], c="k", label=ref_pack.label)
    for res_pack in res_packs:
        ax_3d_trj.plot3D(res_pack.data["x"], res_pack.data["y"], res_pack.data["z"], label=res_pack.label)

    ax_3d_trj.set_title("3D Trajectory", fontsize=opt_param.title_font_size)
    ax_3d_trj.set_xlabel('x[m]', fontsize=opt_param.label_font_size)
    ax_3d_trj.set_ylabel('y[m]', fontsize=opt_param.label_font_size)
    ax_3d_trj.set_zlabel('z[m]', fontsize=opt_param.label_font_size)
    ax_3d_trj.legend()
    ax_3d_trj.grid()

    return {"3d_trajectory": fig_3d_trj}

def plot_xyz(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_xyz = plt.figure("X_Y_Z", figsize=(16, 9), dpi=120)

    for i, name in enumerate(["x", "y", "z"]):
        ax = fig_xyz.add_subplot(311 + i)

        ax.plot(
            ref_pack.axis_array, ref_pack.data[name],
            marker="o", c="k", markersize=3, linewidth=0.5, label=ref_pack.label
        )
        for res_pack in res_packs:
            ax.plot(
                res_pack.axis_array, res_pack.data[name],
                marker="o", markersize=1, linewidth=0.5, label=res_pack.label
            )

        ax.set_title(name.capitalize(), fontsize=opt_param.title_font_size)
        ax.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
        ax.set_ylabel("[m]", fontsize=opt_param.label_font_size)
        ax.tick_params(labelsize=opt_param.ticks_font_size)
        ax.grid()
        ax.legend()
    plt.tight_layout()

    return {"xyz": fig_xyz}

def plot_xyz_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_xyz = plt.figure("X_Y_Z_Error", figsize=(16, 9), dpi=120)

    for i, name in enumerate(["x", "y", "z"]):
        ax = fig_xyz.add_subplot(311 + i)

        for res_pack in res_packs:
            ax.plot(
                res_pack.axis_array, res_pack.data[f"error_{name}"],
                marker="o", markersize=1, linewidth=0.5, label=res_pack.label
            )

        ax.set_title(f"{name.capitalize()} Error", fontsize=opt_param.title_font_size)
        ax.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
        ax.set_ylabel("error[m]", fontsize=opt_param.label_font_size)
        ax.tick_params(labelsize=opt_param.ticks_font_size)
        ax.grid()
        ax.legend()
    plt.tight_layout()

    return {"xyz_error": fig_xyz}

def plot_2d_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_2d_error = plt.figure("2D_Error", figsize=(16, 9), dpi=120)
    ax_2d_error = fig_2d_error.add_subplot(111)

    for res_pack in res_packs:
        ax_2d_error.plot(
            res_pack.axis_array, res_pack.data["error_2d"],
            marker="o", markersize=2, linewidth=0.5, label=res_pack.label
        )

    ax_2d_error.set_title("2D Error", fontsize=opt_param.title_font_size)
    ax_2d_error.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
    ax_2d_error.set_ylabel("error[m]", fontsize=opt_param.label_font_size)
    ax_2d_error.tick_params(labelsize=opt_param.ticks_font_size)
    ax_2d_error.legend()
    ax_2d_error.grid()

    return {"2d_error": fig_2d_error}

def plot_3d_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_3d_error = plt.figure("3D_Error", figsize=(16, 9), dpi=120)
    ax_3d_error = fig_3d_error.add_subplot(111)

    for res_pack in res_packs:
        ax_3d_error.plot(
            res_pack.axis_array, res_pack.data["error_3d"],
            marker="o", markersize=2, linewidth=0.5, label=res_pack.label
        )

    ax_3d_error.set_title("3D Error", fontsize=opt_param.title_font_size)
    ax_3d_error.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
    ax_3d_error.set_ylabel("error[m]", fontsize=opt_param.label_font_size)
    ax_3d_error.tick_params(labelsize=opt_param.ticks_font_size)
    ax_3d_error.legend()
    ax_3d_error.grid()

    return {"3d_error": fig_3d_error}

def plot_longitudinal_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_longitudinal_error = plt.figure("Longitudinal_Error", figsize=(16, 9), dpi=120)
    ax_longitudinal_error = fig_longitudinal_error.add_subplot(111)

    for res_pack in res_packs:
        ax_longitudinal_error.plot(
            res_pack.axis_array, res_pack.data["error_longitudinal"],
            marker="o", markersize=2, linewidth=0.5, label=res_pack.label
        )
        if res_pack.display_ellipse:
            ax_longitudinal_error.plot(
                res_pack.axis_array, res_pack.data["ellipse_longitudinal"],
                marker="o", markersize=2, linewidth=0.5, label=f"{res_pack.label} ellipse"
            )

    ax_longitudinal_error.set_title("Longitudinal Error", fontsize=opt_param.title_font_size)
    ax_longitudinal_error.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
    ax_longitudinal_error.set_ylabel("error[m]", fontsize=opt_param.label_font_size)
    ylim = max(map(abs, ax_longitudinal_error.get_ylim()))
    ax_longitudinal_error.set_ylim(-ylim, ylim)
    ax_longitudinal_error.tick_params(labelsize=opt_param.ticks_font_size)
    ax_longitudinal_error.legend()
    ax_longitudinal_error.grid()

    return {"longitudinal_error": fig_longitudinal_error}

def plot_lateral_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_lateral_error = plt.figure("Lateral_Error", figsize=(16, 9), dpi=120)
    ax_lateral_error = fig_lateral_error.add_subplot(111)

    for res_pack in res_packs:
        ax_lateral_error.plot(
            res_pack.axis_array, res_pack.data["error_lateral"],
            marker="o", markersize=2, linewidth=0.5, label=res_pack.label
        )
        if res_pack.display_ellipse:
            ax_lateral_error.plot(
                res_pack.axis_array, res_pack.data["ellipse_lateral"],
                marker="o", markersize=2, linewidth=0.5, label=f"{res_pack.label} ellipse"
            )

    ax_lateral_error.set_title("Lateral Error", fontsize=opt_param.title_font_size)
    ax_lateral_error.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
    ax_lateral_error.set_ylabel("error[m]", fontsize=opt_param.label_font_size)
    ylim = max(map(abs, ax_lateral_error.get_ylim()))
    ax_lateral_error.set_ylim(-ylim, ylim)
    ax_lateral_error.tick_params(labelsize=opt_param.ticks_font_size)
    ax_lateral_error.legend()
    ax_lateral_error.grid()

    return {"lateral_error": fig_lateral_error}

def plot_rpy(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_rpy = plt.figure("Roll_Pith_Yaw", figsize=(16, 9), dpi=120)

    for i, name in enumerate(["roll", "pitch", "yaw"]):
        ax = fig_rpy.add_subplot(311 + i)

        ax.plot(
            ref_pack.axis_array, ref_pack.degree_formater(ref_pack.data[name]),
            marker="o", c="k", markersize=3, linewidth=0.5, label=ref_pack.label
        )
        for res_pack in res_packs:
            ax.plot(
                res_pack.axis_array, ref_pack.degree_formater(res_pack.data[name]),
                marker="o", markersize=1, linewidth=0.5, label=res_pack.label
            )

        ax.set_title(name.capitalize(), fontsize=opt_param.title_font_size)
        ax.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
        ax.set_ylabel(ref_pack.degree_unit, fontsize=opt_param.label_font_size)
        ylim = max(map(abs, ax.get_ylim()))
        ax.set_ylim(-ylim, ylim)
        ax.tick_params(labelsize=opt_param.ticks_font_size)
        ax.grid()
        ax.legend()
    plt.tight_layout()

    return {"rpy": fig_rpy}

def plot_rpy_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_rpy_error = plt.figure("Roll_Pith_Yaw_Error", figsize=(16, 9), dpi=120)

    for i, name in enumerate(["roll", "pitch", "yaw"]):
        ax = fig_rpy_error.add_subplot(311 + i)

        for res_pack in res_packs:
            ax.plot(
                res_pack.axis_array, ref_pack.degree_formater(res_pack.data[f"error_{name}"]),
                marker="o", markersize=1, linewidth=0.5, label=res_pack.label
            )

        ax.set_title(f"{name.capitalize()} Error", fontsize=opt_param.title_font_size)
        ax.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
        ax.set_ylabel(ref_pack.degree_unit, fontsize=opt_param.label_font_size)
        ylim = max(map(abs, ax.get_ylim()))
        ax.set_ylim(-ylim, ylim)
        ax.tick_params(labelsize=opt_param.ticks_font_size)
        ax.grid()
        ax.legend()
    plt.tight_layout()

    return {"rpy_error": fig_rpy_error}

def plot_velocity(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_velocity = plt.figure("Velocity", figsize=(16, 9), dpi=120)
    ax_velocity = fig_velocity.add_subplot(111)

    ax_velocity.plot(
        ref_pack.axis_array, ref_pack.data["velocity"],
        marker="o", c="k", markersize=3, linewidth=0.5, label=ref_pack.label
    )
    for res_pack in res_packs:
        ax_velocity.plot(
            res_pack.axis_array, res_pack.data["velocity"],
            marker="o", markersize=2, linewidth=0.5, label=res_pack.label
        )

    ax_velocity.set_title("Velocity", fontsize=opt_param.title_font_size)
    ax_velocity.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
    ax_velocity.set_ylabel("velocity[m/s]", fontsize=opt_param.label_font_size)
    ax_velocity.tick_params(labelsize=opt_param.ticks_font_size)
    ax_velocity.legend()
    ax_velocity.grid()

    return {"velocity": fig_velocity}

def plot_velocity_error(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    fig_velocity_error = plt.figure("Velocity_Error", figsize=(16, 9), dpi=120)
    ax_velocity_error = fig_velocity_error.add_subplot(111)

    for res_pack in res_packs:
        ax_velocity_error.plot(
            res_pack.axis_array, res_pack.data["error_velocity"],
            marker="o", markersize=2, linewidth=0.5, label=res_pack.label
        )

    ax_velocity_error.set_title("Velocity Error", fontsize=opt_param.title_font_size)
    ax_velocity_error.set_xlabel(ref_pack.axis_unit, fontsize=opt_param.label_font_size)
    ax_velocity_error.set_ylabel("velocity[m/s]", fontsize=opt_param.label_font_size)
    ax_velocity_error.tick_params(labelsize=opt_param.ticks_font_size)
    ax_velocity_error.legend()
    ax_velocity_error.grid()

    return {"velocity_error": fig_velocity_error}

def plot(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> Dict[str, Figure]:
    figs = reduce(lambda x, y: {**x, **y}, [
        plot_2d_traj(ref_pack, res_packs, opt_param), # 2d trajectory
        plot_3d_traj(ref_pack, res_packs, opt_param), # 3d trajectory
        plot_xyz(ref_pack, res_packs, opt_param), # xyz
        plot_xyz_error(ref_pack, res_packs, opt_param), # xyz error
        plot_2d_error(ref_pack, res_packs, opt_param), # 2d error
        plot_3d_error(ref_pack, res_packs, opt_param), # 3d error
        plot_longitudinal_error(ref_pack, res_packs, opt_param), # longitudinal error
        plot_lateral_error(ref_pack, res_packs, opt_param), # lateral error
        plot_rpy(ref_pack, res_packs, opt_param), # rpy
        plot_rpy_error(ref_pack, res_packs, opt_param), # rpy error
        # plot_velocity(ref_pack, res_packs, opt_param), # velocity
        # plot_velocity_error(ref_pack, res_packs, opt_param), # velocity error
    ])

    plt.show()
    return figs

def save_df(ref_pack: RefDataPack, res_packs: List[ResDataPack], opt_param: OptParam) -> None:
    # save reference df
    ref_cols = ["time", "x", "y", "z", "roll", "pitch", "yaw"]
    ref_pack.data[ref_cols].to_csv(f"{opt_param.output_directory}/sync_{ref_pack.label}_df.csv")
    # save results df
    for res_pack in res_packs:
        res_cols = ref_cols + ([
            "cov_xx","cov_xy","cov_yx","cov_yy",
            "ellipse_long","ellipse_short","ellipse_yaw",
            "ellipse_lateral","ellipse_longitudinal"
        ] if res_pack.display_ellipse else [])
        res_pack.data[res_cols].to_csv(f"{opt_param.output_directory}/sync_{res_pack.label}_df.csv")

def save_figures(figs: Dict[str, Figure], opt_param: OptParam) -> None:
    for name, fig in figs.items():
        fig.savefig(f"{opt_param.output_directory}/{name}.{opt_param.save_extension_type}")

def save(ref_pack: RefDataPack, res_packs: List[ResDataPack], figs: Dict[str, Figure], opt_param: OptParam) -> None:
    # save dataframes
    if opt_param.save_dataframe:
        save_df(ref_pack, res_packs, opt_param)
    # save figures
    if opt_param.save_figures:
        save_figures(figs, opt_param)
