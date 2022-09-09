import math

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np


# fmt: off
def output_graph(ref_param, result_param, output_dir, save_param, op_param):
    time = ref_param.df["time"] - ref_param.df["time"][0]
    error_x = result_param.df["x"] - ref_param.df["x"]
    error_y = result_param.df["y"] - ref_param.df["y"]
    error_z = result_param.df["z"] - ref_param.df["z"]
    error_roll = result_param.df["roll"] - ref_param.df["roll"]
    error_pitch = result_param.df["pitch"] - ref_param.df["pitch"]
    error_yaw = result_param.df["yaw"] - ref_param.df["yaw"]

    r_absmax = max(ref_param.df["roll"].abs().max(axis=0), result_param.df["roll"].abs().max(axis=0))
    p_absmax = max(ref_param.df["pitch"].abs().max(axis=0), result_param.df["pitch"].abs().max(axis=0))
    y_absmax = max(ref_param.df["yaw"].abs().max(axis=0), result_param.df["yaw"].abs().max(axis=0))
    er_absmax = error_roll.abs().max(axis=0)
    ep_absmax = error_pitch.abs().max(axis=0)
    ey_absmax = error_yaw.abs().max(axis=0)


    # 2D Trajectory
    fig_2d_trj = plt.figure("2D_Trajectory", figsize=(16, 9), dpi=120)
    ax_2d_trj = fig_2d_trj.add_subplot(111)
    ax_2d_trj.set_title("2D Trajectory", fontsize=save_param.title_font_size)
    ax_2d_trj.scatter(ref_param.df["x"], ref_param.df["y"], c="k", label="Reference")
    ax_2d_trj.scatter(result_param.df["x"], result_param.df["y"], c="r", s=2, label="Result")
    ax_2d_trj.plot([ref_param.df["x"], result_param.df["x"]], [ref_param.df["y"], result_param.df["y"]], "g-", linewidth=0.2, zorder=1)
    if op_param.display_ellipse == True:
        for i in range(1,len(result_param.df)):
            e = patches.Ellipse(xy=(result_param.df['x'][i],result_param.df['y'][i]), width=result_param.df["ellipse_long"][i]*2, height=result_param.df["ellipse_short"][i]*2, angle=math.degrees(result_param.df["ellipse_yaw"][i]), alpha=0.3,color='m') 
            ax_2d_trj.add_patch(e)
    ax_2d_trj.set_xlabel("x[m]", fontsize=save_param.label_font_size)
    ax_2d_trj.set_ylabel("y[m]", fontsize=save_param.label_font_size)
    ax_2d_trj.tick_params(labelsize=save_param.ticks_font_size)
    ax_2d_trj.legend()
    ax_2d_trj.set_aspect("equal")
    ax_2d_trj.grid()

    # 3D Trajectory
    # fig_3d_trj = plt.figure('3D_Trajectory')
    # ax_3d_trj = fig_3d_trj.add_subplot(projection='3d')
    # ax_3d_trj.set_title('3D Trajectory')
    # ax_3d_trj.plot3D(ref_param.df['x'], ref_param.df['y'], ref_param.df['z'], c = "k")
    # ax_3d_trj.plot3D(result_param.df['x'], result_param.df['y'], result_param.df['z'], c="r")
    # ax_3d_trj.set_xlabel('x [m]')
    # ax_3d_trj.set_ylabel('y [m]')
    # ax_3d_trj.set_zlabel('z [m]')
    # ax_3d_trj.grid()

    # 2D Error
    error_2d = np.sqrt(pow(error_x, 2) + pow(error_y, 2))
    fig_2d_error = plt.figure("2D_Error", figsize=(16, 9), dpi=120)
    ax_2d_error = fig_2d_error.add_subplot(111)
    ax_2d_error.set_title("2D Error", fontsize=save_param.title_font_size)
    ax_2d_error.plot(time, error_2d, marker="o", c="k", markersize=2, linewidth = 0.5)
    ax_2d_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_2d_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # x_min, x_max = ax_2d_error.get_xlim()
    # ax_2d_error.set_xlim(0, x_max)
    y_min, y_max = ax_2d_error.get_ylim()
    ax_2d_error.set_ylim(0, y_max)
    ax_2d_trj.tick_params(labelsize=save_param.ticks_font_size)
    ax_2d_error.grid()
    del error_2d

    # Hight Error
    fig_hight_error = plt.figure("Hight_Error", figsize=(16, 9), dpi=120)
    ax_hight_error = fig_hight_error.add_subplot(111)
    ax_hight_error.set_title("Hight Error", fontsize=save_param.title_font_size)
    ax_hight_error.plot(time, error_z, marker="o", c="k", markersize=2, linewidth = 0.5)
    ax_hight_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_hight_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_hight_error.set_xlim(0, x_max)
    y_min, y_max = ax_hight_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_hight_error.set_ylim(-y_max, y_max)
    ax_hight_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_hight_error.grid()

    # 3D Error
    error_3d = np.sqrt(pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2))
    fig_3d_error = plt.figure("3D_Error", figsize=(16, 9), dpi=120)
    ax_3d_error = fig_3d_error.add_subplot(111)
    ax_3d_error.set_title("3D Error", fontsize=save_param.title_font_size)
    ax_3d_error.plot(time, error_3d, marker="o", c="k", markersize=2, linewidth = 0.5)
    ax_3d_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_3d_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_3d_error.set_xlim(0, x_max)
    y_min, y_max = ax_3d_error.get_ylim()
    ax_3d_error.set_ylim(0, y_max)
    ax_3d_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_3d_error.grid()
    del error_3d

    # Error considering vehicle direction
    longitudinal = []
    lateral = []
    for i in range(len(ref_param.df)):
        longitudinal.append(error_x[i] * math.cos(-ref_param.df["yaw"][i]) - error_y[i] * math.sin(-ref_param.df["yaw"][i]))
        lateral.append(error_x[i] * math.sin(-ref_param.df["yaw"][i]) + error_y[i] * math.cos(-ref_param.df["yaw"][i]))
    # Longitudinal Error
    fig_longitudinal_error = plt.figure("Longitudinal_Error", figsize=(16, 9), dpi=120)
    ax_longitudinal_error = fig_longitudinal_error.add_subplot(111)
    ax_longitudinal_error.set_title("Longitudinal Error", fontsize=save_param.title_font_size)
    ax_longitudinal_error.plot(time, longitudinal, marker="o", c="k", markersize=2, linewidth = 0.5)
    if op_param.display_ellipse == True:
        ax_longitudinal_error.plot(time, result_param.df["ellipse_longitudinal"], marker="o", c="m", markersize=2, linewidth = 0.5)
    ax_longitudinal_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_longitudinal_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_longitudinal_error.set_xlim(0, x_max)
    y_min, y_max = ax_longitudinal_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_longitudinal_error.set_ylim(-y_max, y_max)
    # ax_longitudinal_error.set_yticks(np.arange(-6, 6.1, 1))
    ax_longitudinal_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_longitudinal_error.grid()

    # Lateral Error
    fig_lateral_error = plt.figure("Lateral_Error", figsize=(16, 9), dpi=120)
    ax_lateral_error = fig_lateral_error.add_subplot(111)
    ax_lateral_error.set_title("Lateral Error", fontsize=save_param.title_font_size)
    ax_lateral_error.plot(time, lateral, marker="o", c="k", markersize=2, linewidth = 0.5)
    if op_param.display_ellipse == True:
        ax_lateral_error.plot(time, result_param.df["ellipse_lateral"], marker="o", c="m", markersize=2, linewidth = 0.5)
    ax_lateral_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_lateral_error.set_ylabel("error[m]", fontsize=save_param.label_font_size)
    # ax_lateral_error.set_xlim(0, x_max)
    y_min, y_max = ax_lateral_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_lateral_error.set_ylim(-y_max, y_max)
    # ax_lateral_error.set_yticks(np.arange(-1.0, 1.1, 0.1))
    ax_lateral_error.tick_params(labelsize=save_param.ticks_font_size)
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
    ax_r.plot(time, ref_param.df["roll"] * rad_to_deg, marker="o", c="k", markersize=3, linewidth = 0.5, label="Reference")
    ax_r.plot(time, result_param.df["roll"] * rad_to_deg, marker="o", c="r", markersize=1, linewidth = 0.5, label="Result")
    ax_r.set_xlabel("time[s]", fontsize=save_param.label_font_size)
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
    ax_p.plot(time, ref_param.df["pitch"] * rad_to_deg, marker="o", c="k", markersize=3, linewidth = 0.5, label="Reference")
    ax_p.plot(time, result_param.df["pitch"] * rad_to_deg, marker="o", c="r", markersize=1, linewidth = 0.5, label="Result")
    ax_p.set_xlabel("time[s]", fontsize=save_param.label_font_size)
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
    ax_y.plot(time, ref_param.df["yaw"] * rad_to_deg, marker="o", c="k", markersize=3, linewidth = 0.5, label="Reference")
    ax_y.plot(time, result_param.df["yaw"] * rad_to_deg, marker="o", c="r", markersize=1, linewidth = 0.5, label="Result")
    ax_y.set_xlabel("time[s]", fontsize=save_param.label_font_size)
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
    ax_roll_error.plot(time, error_roll * rad_to_deg, marker="o", c="k", markersize=2, linewidth = 0.5)
    ax_roll_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_roll_error.set_ylabel("error" + rpy_label, fontsize=save_param.label_font_size)
    # ax_roll_error.set_xlim(0, x_max)
    y_min, y_max = ax_roll_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_roll_error.set_ylim(-y_max, y_max)
    ax_roll_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_roll_error.grid()

    # Pitch Error
    ax_pitch_error = fig_rpy_error.add_subplot(312)
    ax_pitch_error.set_title("Pitch_Error", fontsize=save_param.title_font_size)
    ax_pitch_error.plot(time, error_pitch * rad_to_deg, marker="o", c="k", markersize=2, linewidth = 0.5)
    ax_pitch_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_pitch_error.set_ylabel("error" + rpy_label, fontsize=save_param.label_font_size)
    # ax_pitch_error.set_xlim(0, x_max)
    y_min, y_max = ax_pitch_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_pitch_error.set_ylim(-y_max, y_max)
    ax_pitch_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_pitch_error.grid()

    # Yaw Error
    ax_yaw_error = fig_rpy_error.add_subplot(313)
    ax_yaw_error.set_title("Yaw_Error", fontsize=save_param.title_font_size)
    ax_yaw_error.plot(time, error_yaw * rad_to_deg, marker="o", c="k", markersize=2, linewidth = 0.5)
    ax_yaw_error.set_xlabel("time[s]", fontsize=save_param.label_font_size)
    ax_yaw_error.set_ylabel("error" + rpy_label, fontsize=save_param.label_font_size)
    # ax_yaw_error.set_xlim(0, x_max)
    y_min, y_max = ax_yaw_error.get_ylim()
    y_max = max(abs(y_min),abs(y_max))
    ax_yaw_error.set_ylim(-y_max, y_max)
    ax_yaw_error.tick_params(labelsize=save_param.ticks_font_size)
    ax_yaw_error.grid()
    
    plt.tight_layout()
    
    # Velocity

    # Velocity Error

    # Exection time

    print("Completed!!")

    # Save Dataframe
    if save_param.save_dataframe == True:
        print("Now saving csv files ...", end="")
        ref_param.df.to_csv(output_dir + "/sync_ref_df.csv")
        result_param.df.to_csv(output_dir + "/sync_result_df.csv")
        print("Completed!!")

    # Save Figures
    if save_param.save_figures == True:
        print("Now saving figures ...", end="")
        fig_2d_trj.savefig(output_dir + "/2d_trj." + save_param.save_extension_type)
        # fig_3d_trj.savefig(output_dir + "/3d_trj." + save_param.save_extension_type)
        fig_2d_error.savefig(output_dir + "/2d_error." + save_param.save_extension_type)
        fig_hight_error.savefig(output_dir + "/hight_error." + save_param.save_extension_type)
        fig_3d_error.savefig(output_dir + "/3d_error." + save_param.save_extension_type)
        fig_longitudinal_error.savefig(output_dir + "/longitudinal_error." + save_param.save_extension_type)
        fig_lateral_error.savefig(output_dir + "/lateral_error." + save_param.save_extension_type)
        fig_rpy.savefig(output_dir + "/rpy." + save_param.save_extension_type)
        fig_rpy_error.savefig(output_dir + "/rpy_error." + save_param.save_extension_type)
        print("Completed!!")

    plt.show()
    # fmt: on
