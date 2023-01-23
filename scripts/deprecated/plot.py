import math

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import axes3d, Axes3D 


# fmt: off
def output_graph(ref_param, result_params, save_param, op_param):
    # Cumulative time
    time = ref_param.df["time"] - ref_param.df["time"][0]

    # Cumulative distance
    if save_param.axis_type == 1 or save_param.progress_info == 4:
        displacement = [0] + [
            np.sqrt(pow(dx, 2) + pow(dy, 2)) for dx, dy in zip(
                ref_param.df["x"][1:].values - ref_param.df["x"][:-1].values,
                ref_param.df["y"][1:].values - ref_param.df["y"][:-1].values,
            )
        ]
        distance = [sum(displacement[:idx + 1]) for idx in range(len(displacement))]

    # Choose axis type
    axis_array = []
    if save_param.axis_type == 0:
        axis_array = time
        axis_unit = "time [s]"
    else:
        axis_array = distance
        axis_unit = "distance [m]"

    # Calc velocity
    for param in [ref_param] + result_params:
        param.velocity = np.array([
            np.sqrt(pow(dx, 2) + pow(dy, 2)) / dt
            for dx, dy, dt in zip(
                param.df["x"][1:].values - param.df["x"][:-1].values,
                param.df["y"][1:].values - param.df["y"][:-1].values,
                param.df["time"][1:].values - param.df["time"][:-1].values
            )
        ])

    # Calc error
    round2pipi = lambda v: (v % (math.pi * 2)) - (0 if (v % (math.pi * 2)) < math.pi else (math.pi * 2))
    for result_param in result_params:
        result_param.error_x = result_param.df["x"] - ref_param.df["x"]
        result_param.error_y = result_param.df["y"] - ref_param.df["y"]
        result_param.error_z = result_param.df["z"] - ref_param.df["z"]
        result_param.error_roll = (result_param.df["roll"] - ref_param.df["roll"]).map(round2pipi)
        result_param.error_pitch = (result_param.df["pitch"] - ref_param.df["pitch"]).map(round2pipi)
        result_param.error_yaw = (result_param.df["yaw"] - ref_param.df["yaw"]).map(round2pipi)
        result_param.error_velocity = result_param.velocity - ref_param.velocity


    # r_absmax = max(ref_param.df["roll"].abs().max(axis=0), result_param.df["roll"].abs().max(axis=0))
    # p_absmax = max(ref_param.df["pitch"].abs().max(axis=0), result_param.df["pitch"].abs().max(axis=0))
    # y_absmax = max(ref_param.df["yaw"].abs().max(axis=0), result_param.df["yaw"].abs().max(axis=0))
    # er_absmax = error_roll.abs().max(axis=0)
    # ep_absmax = error_pitch.abs().max(axis=0)
    # ey_absmax = error_yaw.abs().max(axis=0)


    # 2D Trajectory
    fig_2d_trj = plt.figure("2D_Trajectory", figsize=(16, 9), dpi=120)
    ax_2d_trj = fig_2d_trj.add_subplot(111)
    ax_2d_trj.set_title("2D Trajectory", fontsize=save_param.title_font_size)
    for param in [ref_param] + result_params: # dilute trajectory data for better performance
        param.df_temp, param.df = param.df, param.df.iloc[::save_param.dilution_step]
    ax_2d_trj.scatter(ref_param.df["x"], ref_param.df["y"], c="k", label=ref_param.label)
    for result_param in result_params:
        ax_2d_trj.scatter(result_param.df["x"], result_param.df["y"], s=2, label=result_param.label)
        ax_2d_trj.plot([ref_param.df["x"], result_param.df["x"]], [ref_param.df["y"], result_param.df["y"]], "-", linewidth=0.2, zorder=1)
        if op_param.display_ellipse == True:
            for i in range(len(result_param.df)):
                e = patches.Ellipse(xy=(result_param.df['x'][i],result_param.df['y'][i]), width=result_param.df["ellipse_long"][i]*2, height=result_param.df["ellipse_short"][i]*2, angle=math.degrees(result_param.df["ellipse_yaw"][i]), alpha=0.3,color='m') 
                ax_2d_trj.add_patch(e)

        if save_param.progress_info != 0:
            c = 1
            for i in range(1, len(result_param.df)):
                if save_param.progress_info == 1 and i >= save_param.interval * c: # number
                    ax_2d_trj.text(result_param.df["x"][i], result_param.df["y"][i], i, va='bottom')
                    c += 1
                elif save_param.progress_info == 2 and time[i] >= save_param.interval * c: # time
                    ax_2d_trj.text(result_param.df["x"][i], result_param.df["y"][i], round(time[i],3), va='bottom')
                    c += 1
                elif save_param.progress_info == 3 and time[i] >= save_param.interval * c: #rostime
                    ax_2d_trj.text(result_param.df["x"][i], result_param.df["y"][i], ref_param.df["bottom"][i], va='bottom')
                    c += 1
                elif save_param.progress_info == 4 and distance[i] >= save_param.interval * c: # distance
                    ax_2d_trj.text(result_param.df["x"][i], result_param.df["y"][i], round(distance[i],1), va='bottom')
                    c += 1
    for param in [ref_param] + result_params:
        param.df = param.df_temp
        del param.df_temp

    ax_2d_trj.set_xlabel("x[m]", fontsize=save_param.label_font_size)
    ax_2d_trj.set_ylabel("y[m]", fontsize=save_param.label_font_size)
    ax_2d_trj.tick_params(labelsize=save_param.ticks_font_size)
    ax_2d_trj.legend()
    ax_2d_trj.set_aspect("equal")
    ax_2d_trj.grid()

    # 3D Trajectory
    fig_3d_trj = plt.figure('3D_Trajectory', figsize=(16, 9), dpi=120)
    ax_3d_trj = fig_3d_trj.add_subplot(projection='3d')
    ax_3d_trj.set_title('3D Trajectory')
    ax_3d_trj.plot3D(ref_param.df["x"], ref_param.df["y"], ref_param.df["z"], c = "k")
    for result_param in result_params:
        ax_3d_trj.plot3D(result_param.df["x"], result_param.df["y"], result_param.df["z"])
    ax_3d_trj.set_xlabel('x[m]')
    ax_3d_trj.set_ylabel('y[m]')
    ax_3d_trj.set_zlabel('z[m]')
    ax_3d_trj.grid()

    # 2D Error
    fig_2d_error = plt.figure("2D_Error", figsize=(16, 9), dpi=120)
    ax_2d_error = fig_2d_error.add_subplot(111)
    ax_2d_error.set_title("2D Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        error_2d = np.sqrt(pow(result_param.error_x, 2) + pow(result_param.error_y, 2))
        ax_2d_error.plot(axis_array, error_2d, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_2d_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_2d_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # x_min, x_max = ax_2d_error.get_xlim()
    # ax_2d_error.set_xlim(0, x_max)
    y_min, y_max = ax_2d_error.get_ylim()
    ax_2d_error.set_ylim(0, y_max)
    ax_2d_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_2d_error.legend()
    ax_2d_error.grid()
    del error_2d

    # Height Error
    fig_height_error = plt.figure("Height_Error", figsize=(16, 9), dpi=120)
    ax_height_error = fig_height_error.add_subplot(111)
    ax_height_error.set_title("Height Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        ax_height_error.plot(axis_array, result_param.error_z, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_height_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_height_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_height_error.set_xlim(0, x_max)
    y_min, y_max = ax_height_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_height_error.set_ylim(-y_max, y_max)
    ax_height_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_height_error.legend()
    ax_height_error.grid()

    # 3D Error
    fig_3d_error = plt.figure("3D_Error", figsize=(16, 9), dpi=120)
    ax_3d_error = fig_3d_error.add_subplot(111)
    ax_3d_error.set_title("3D Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        error_3d = np.sqrt(pow(result_param.error_x, 2) + pow(result_param.error_y, 2) + pow(result_param.error_z, 2))
        ax_3d_error.plot(axis_array, error_3d, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_3d_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_3d_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_3d_error.set_xlim(0, x_max)
    y_min, y_max = ax_3d_error.get_ylim()
    ax_3d_error.set_ylim(0, y_max)
    ax_3d_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_3d_error.legend()
    ax_3d_error.grid()
    del error_3d

    # Error considering vehicle direction
    # Longitudinal Error
    fig_longitudinal_error = plt.figure("Longitudinal_Error", figsize=(16, 9), dpi=120)
    ax_longitudinal_error = fig_longitudinal_error.add_subplot(111)
    ax_longitudinal_error.set_title("Longitudinal Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        longitudinal = [result_param.error_x[i] * math.cos(-ref_param.df["yaw"][i]) - result_param.error_y[i] * math.sin(-ref_param.df["yaw"][i])
            for i in range(len(ref_param.df))]
        ax_longitudinal_error.plot(axis_array, longitudinal, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
        if op_param.display_ellipse == True:
            ax_longitudinal_error.plot(axis_array, result_param.df["ellipse_longitudinal"], marker="o", markersize=2, linewidth=0.5, label=result_param.label + 'ellipse')
    ax_longitudinal_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_longitudinal_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_longitudinal_error.set_xlim(0, x_max)
    y_min, y_max = ax_longitudinal_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_longitudinal_error.set_ylim(-y_max, y_max)
    # ax_longitudinal_error.set_yticks(np.arange(-6, 6.1, 1))
    ax_longitudinal_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_longitudinal_error.legend()
    ax_longitudinal_error.grid()

    # Lateral Error
    fig_lateral_error = plt.figure("Lateral_Error", figsize=(16, 9), dpi=120)
    ax_lateral_error = fig_lateral_error.add_subplot(111)
    ax_lateral_error.set_title("Lateral Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        lateral = [result_param.error_x[i] * math.sin(-ref_param.df["yaw"][i]) + result_param.error_y[i] * math.cos(-ref_param.df["yaw"][i])
            for i in range(len(ref_param.df))]
        ax_lateral_error.plot(axis_array, lateral, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
        if op_param.display_ellipse == True:
            ax_lateral_error.plot(axis_array, result_param.df["ellipse_lateral"], marker="o", markersize=2, linewidth=0.5, label=result_param.label + 'ellipse')
    ax_lateral_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_lateral_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_lateral_error.set_xlim(0, x_max)
    y_min, y_max = ax_lateral_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_lateral_error.set_ylim(-y_max, y_max)
    # ax_lateral_error.set_yticks(np.arange(-1.0, 1.1, 0.1))
    ax_lateral_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_lateral_error.legend()
    ax_lateral_error.grid()

    # Distance to the Nearest Reference Pose

    # To degree
    if save_param.display_radian == False:
        rad_to_deg = 180 / math.pi
        rpy_label = "[degree]"
    else:
        rad_to_deg = 1
        rpy_label = "[rad]"

    # Reference and Result Roll Pitch Yaw
    fig_rpy = plt.figure("Roll_Pitch_Yaw", figsize=(16, 9), dpi=120)
    # Roll
    ax_r = fig_rpy.add_subplot(311)
    ax_r.set_title("Roll", fontsize=save_param.title_font_size)
    ax_r.plot(axis_array, ref_param.df["roll"] * rad_to_deg, marker="o", c="k", markersize=3, linewidth=0.5, label=ref_param.label)
    for result_param in result_params:
        ax_r.plot(axis_array, result_param.df["roll"] * rad_to_deg, marker="o", markersize=1, linewidth=0.5, label=result_param.label)
    ax_r.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_r.set_ylabel(rpy_label, fontsize=save_param.label_font_size)
    # ax_r.set_xlim(0, x_max)
    y_min, y_max = ax_r.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_r.set_ylim(-y_max, y_max)
    ax_r.tick_params(labelsize=save_param.ticks_font_size)
    ax_r.grid()
    ax_r.legend()

    # Pitch
    ax_p = fig_rpy.add_subplot(312)
    ax_p.set_title("Pitch", fontsize=save_param.title_font_size)
    ax_p.plot(axis_array, ref_param.df["pitch"] * rad_to_deg, marker="o", c="k", markersize=3, linewidth=0.5, label=ref_param.label)
    for result_param in result_params:
        ax_p.plot(axis_array, result_param.df["pitch"] * rad_to_deg, marker="o", markersize=1, linewidth=0.5, label=result_param.label)
    ax_p.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_p.set_ylabel(rpy_label, fontsize=save_param.label_font_size)
    # ax_p.set_xlim(0, x_max)
    y_min, y_max = ax_p.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_p.set_ylim(-y_max, y_max)
    ax_p.tick_params(labelsize=save_param.ticks_font_size)
    ax_p.grid()
    ax_p.legend()

    # Yaw
    ax_y = fig_rpy.add_subplot(313)
    ax_y.set_title("Yaw", fontsize=save_param.title_font_size)
    ax_y.plot(axis_array, ref_param.df["yaw"] * rad_to_deg, marker="o", c="k", markersize=3, linewidth=0.5, label=ref_param.label)
    for result_param in result_params:
        ax_y.plot(axis_array, result_param.df["yaw"] * rad_to_deg, marker="o", markersize=1, linewidth=0.5, label=result_param.label)
    ax_y.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_y.set_ylabel(rpy_label, fontsize=save_param.label_font_size)
    # ax_y.set_xlim(0, x_max)
    y_min, y_max = ax_y.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_y.set_ylim(-y_max, y_max)
    ax_y.tick_params(labelsize=save_param.ticks_font_size)
    ax_y.grid()
    ax_y.legend()

    plt.tight_layout()

    # Roll Pitch Yaw Error
    fig_rpy_error = plt.figure("Roll_Pitch_Yaw_Error", figsize=(16, 9), dpi=120)
    # Roll Error
    ax_roll_error = fig_rpy_error.add_subplot(311)
    ax_roll_error.set_title("Roll_Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        ax_roll_error.plot(axis_array, result_param.error_roll * rad_to_deg, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_roll_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_roll_error.set_ylabel("error" + rpy_label, fontsize=save_param.label_font_size)
    # ax_roll_error.set_xlim(0, x_max)
    y_min, y_max = ax_roll_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_roll_error.set_ylim(-y_max, y_max)
    ax_roll_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_roll_error.grid()
    ax_roll_error.legend()

    # Pitch Error
    ax_pitch_error = fig_rpy_error.add_subplot(312)
    ax_pitch_error.set_title("Pitch_Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        ax_pitch_error.plot(axis_array, result_param.error_pitch * rad_to_deg, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_pitch_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_pitch_error.set_ylabel("error" + rpy_label, fontsize=save_param.label_font_size)
    # ax_pitch_error.set_xlim(0, x_max)
    y_min, y_max = ax_pitch_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_pitch_error.set_ylim(-y_max, y_max)
    ax_pitch_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_pitch_error.grid()
    ax_pitch_error.legend()

    # Yaw Error
    ax_yaw_error = fig_rpy_error.add_subplot(313)
    ax_yaw_error.set_title("Yaw_Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        ax_yaw_error.plot(axis_array, result_param.error_yaw * rad_to_deg, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_yaw_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_yaw_error.set_ylabel("error" + rpy_label, fontsize=save_param.label_font_size)
    # ax_yaw_error.set_xlim(0, x_max)
    y_min, y_max = ax_yaw_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_yaw_error.set_ylim(-y_max, y_max)
    ax_yaw_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_yaw_error.grid()
    ax_yaw_error.legend()
    
    plt.tight_layout()
    
    # Velocity
    fig_velocity = plt.figure("Velocity", figsize=(16, 9), dpi=120)
    ax_velocity = fig_velocity.add_subplot(111)
    ax_velocity.set_title("Velocity", fontsize=save_param.title_font_size)
    ax_velocity.plot(axis_array[:-1], ref_param.velocity, c="k", marker="o", markersize=3, linewidth=0.5, label=ref_param.label)
    for result_param in result_params:
        ax_velocity.plot(axis_array[:-1], result_param.velocity, marker="o", markersize=1, linewidth=0.5, label=result_param.label)
    ax_velocity.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_velocity.set_ylabel("[m/s]", fontsize=save_param.label_font_size)
    y_min, y_max = ax_velocity.get_ylim()
    y_max = max(abs(y_min), abs(y_max))
    ax_velocity.set_ylim(-y_max / 2, y_max)
    ax_velocity.tick_params(labelsize=save_param.ticks_font_size)
    ax_velocity.legend()
    ax_velocity.grid()

    # Velocity Error
    fig_velocity_error = plt.figure("Velocity_Error", figsize=(16, 9), dpi=120)
    ax_velocity_error = fig_velocity_error.add_subplot(111)
    ax_velocity_error.set_title("Velocity Error", fontsize=save_param.title_font_size)
    for result_param in result_params:
        ax_velocity_error.plot(axis_array[:-1], result_param.error_velocity, marker="o", markersize=2, linewidth=0.5, label=result_param.label)
    ax_velocity_error.set_xlabel(axis_unit, fontsize=save_param.label_font_size)
    ax_velocity_error.set_ylabel("error[m/s]", fontsize=save_param.label_font_size)
    y_min, y_max = ax_velocity_error.get_ylim()
    y_max = max(abs(y_min), abs(y_max))
    ax_velocity_error.set_ylim(-y_max, y_max)
    ax_velocity_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_velocity_error.legend()
    ax_velocity_error.grid()

    # Exection time

    print("Completed!!")

    # Save Dataframe
    if save_param.save_dataframe == True:
        print("Now saving csv files ...", end="")
        ref_param.df = ref_param.df[["time","x","y","z","roll","pitch","yaw"]]
        ref_param.df.to_csv(save_param.output_directory + f"/sync_{ref_param.label}_df.csv")
        for result_param in result_params:
            if op_param.display_ellipse == True:
                result_param.df = result_param.df[["time","x","y","z","roll","pitch","yaw","cov_xx","cov_xy","cov_yx","cov_yy",
                    "ellipse_long","ellipse_short","ellipse_yaw","ellipse_lateral","ellipse_longitudinal"]]
            else:
                result_param.df = result_param.df[["time","x","y","z","roll","pitch","yaw"]]
            result_param.df.to_csv(save_param.output_directory + f"/sync_{result_param.label}_df.csv")
        print("Completed!!")

    # Save Figures
    if save_param.save_figures == True:
        print("Now saving figures ...", end="")
        fig_2d_trj.savefig(save_param.output_directory + "/2d_trj." + save_param.save_extension_type)
        fig_3d_trj.savefig(save_param.output_directory + "/3d_trj." + save_param.save_extension_type)
        fig_2d_error.savefig(save_param.output_directory + "/2d_error." + save_param.save_extension_type)
        fig_height_error.savefig(save_param.output_directory + "/height_error." + save_param.save_extension_type)
        fig_3d_error.savefig(save_param.output_directory + "/3d_error." + save_param.save_extension_type)
        fig_longitudinal_error.savefig(save_param.output_directory + "/longitudinal_error." + save_param.save_extension_type)
        fig_lateral_error.savefig(save_param.output_directory + "/lateral_error." + save_param.save_extension_type)
        fig_rpy.savefig(save_param.output_directory + "/rpy." + save_param.save_extension_type)
        fig_rpy_error.savefig(save_param.output_directory + "/rpy_error." + save_param.save_extension_type)
        fig_velocity.savefig(save_param.output_directory + "/velocity." + save_param.save_extension_type)
        fig_velocity_error.savefig(save_param.output_directory + "/velocity_error." + save_param.save_extension_type)
        print("Completed!!")

    plt.show()
    # fmt: on
