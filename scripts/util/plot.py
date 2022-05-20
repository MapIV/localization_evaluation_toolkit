import matplotlib.pyplot as plt
import math
import numpy as np

def output_graph(sync_ref_df, sync_result_df, output_dir, save_param):
    time = sync_ref_df['ref_time'] - sync_ref_df['ref_time'][0]
    error_x = sync_result_df['result_x'] - sync_ref_df['ref_x']
    error_y = sync_result_df['result_y'] - sync_ref_df['ref_y']
    error_z = sync_result_df['result_z'] - sync_ref_df['ref_z']
    
    # 2D Trajectory
    fig_2d_trj = plt.figure('2D_Trajectory')
    ax_2d_trj = fig_2d_trj.add_subplot(111)
    ax_2d_trj.set_title('2D Trajectory')
    ax_2d_trj.scatter(sync_ref_df['ref_x'], sync_ref_df['ref_y'], c="k")
    ax_2d_trj.scatter(sync_result_df['result_x'], sync_result_df['result_y'], c="r", s=2)
    ax_2d_trj.plot([sync_ref_df['ref_x'],sync_result_df['result_x']],[sync_ref_df['ref_y'],sync_result_df['result_y']],"g-",linewidth=0.2,zorder=1)
    ax_2d_trj.set_xlabel('x[m]')
    ax_2d_trj.set_ylabel('y[m]')
    ax_2d_trj.set_aspect('equal')
    ax_2d_trj.grid()
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # 3D Trajectory
    fig_3d_trj = plt.figure('3D_Trajectory')
    ax_3d_trj = fig_3d_trj.add_subplot(projection='3d')
    ax_3d_trj.set_title('3D Trajectory')
    ax_3d_trj.plot3D(sync_ref_df['ref_x'], sync_ref_df['ref_y'], sync_ref_df['ref_z'], c = "k")
    ax_3d_trj.plot3D(sync_result_df['result_x'], sync_result_df['result_y'], sync_result_df['result_z'], c="r")
    ax_3d_trj.set_xlabel('x [m]')
    ax_3d_trj.set_ylabel('y [m]')
    ax_3d_trj.set_zlabel('z [m]')
    ax_3d_trj.grid()
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # Reference and Result Roll Pitch Yaw
    fig_rpy = plt.figure('Roll_Pitch_Yaw', constrained_layout=True)
    ax_ref_r = fig_rpy.add_subplot(321)
    ax_ref_p = fig_rpy.add_subplot(323)
    ax_ref_y = fig_rpy.add_subplot(325)
    ax_result_r = fig_rpy.add_subplot(322)
    ax_result_p = fig_rpy.add_subplot(324)
    ax_resukt_y = fig_rpy.add_subplot(326)

    ax_ref_r.plot(time, sync_ref_df['ref_roll'], marker="o", markersize=4)
    ax_ref_p.plot(time, sync_ref_df['ref_pitch'], marker="o")
    ax_ref_y.plot(time, sync_ref_df['ref_yaw'], marker="o")
    ax_result_r.plot(time, sync_result_df['result_roll'], marker="o")
    ax_result_p.plot(time, sync_result_df['result_pitch'], marker="o")
    ax_resukt_y.plot(time, sync_result_df['result_yaw'], marker="o")
    
    ax_ref_r.set_xlabel('time[s]')
    ax_ref_r.set_ylabel('[deg]')
    ax_ref_r.set_title('Reference Roll', fontsize = 14, fontweight='bold')
    ax_ref_r.grid()

    ax_ref_p.set_xlabel('time[s]')
    ax_ref_p.set_ylabel('[deg]')
    ax_ref_p.set_title('Reference Pitch', fontsize = 14, fontweight='bold')
    ax_ref_p.grid()

    ax_ref_y.set_xlabel('time[s]')
    ax_ref_y.set_ylabel('[deg]')
    ax_ref_y.set_title('Reference Yaw', fontsize = 14, fontweight='bold')
    ax_ref_y.grid()

    ax_result_r.set_xlabel('time[s]')
    ax_result_r.set_ylabel('[deg]')
    ax_result_r.set_title('Result Roll', fontsize = 14, fontweight='bold')
    ax_result_r.grid()

    ax_result_p.set_xlabel('time[s]')
    ax_result_p.set_ylabel('[deg]')
    ax_result_p.set_title('Result Pitch', fontsize = 14, fontweight='bold')
    ax_result_p.grid()

    ax_resukt_y.set_xlabel('time[s]')
    ax_resukt_y.set_ylabel('[deg]')
    ax_resukt_y.set_title('Result Yaw', fontsize = 14, fontweight='bold')
    ax_resukt_y.grid()

    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # Velocity

    # 2D Error
    error_2d = np.sqrt(pow(error_x, 2) + pow(error_y, 2))
    fig_2d_error = plt.figure('2D_Error')
    ax_2d_error = fig_2d_error.add_subplot(111)
    ax_2d_error.plot(time, error_2d, marker="o", c='k', markersize=2)
    ax_2d_error.set_xlabel('time[s]')
    ax_2d_error.set_ylabel('error[m]')
    ax_2d_error.set_title('2D Error')
    ax_2d_error.grid()
    del error_2d
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # Hight Error
    fig_hight_error = plt.figure('Hight_Error')
    ax_hight_error = fig_hight_error.add_subplot(111)
    ax_hight_error.plot(time, error_z, marker="o", c='k', markersize=2)
    ax_hight_error.set_xlabel('time[s]')
    ax_hight_error.set_ylabel('error[m]')
    ax_hight_error.set_title('Hight Error')
    ax_hight_error.grid()
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # 3D Error
    error_3d = np.sqrt(pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2))
    fig_3d_error = plt.figure('3D_Error')
    ax_3d_error = fig_3d_error.add_subplot(111)
    ax_3d_error.plot(time, error_3d, marker="o", c='k', markersize=2)
    ax_3d_error.set_xlabel('time[s]')
    ax_3d_error.set_ylabel('error[m]')
    ax_3d_error.set_title('3D Error')
    ax_3d_error.grid()
    del error_3d
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()
    
    # Error considering vehicle direction
    longitudinal = []
    lateral = []
    for i in range(len(sync_ref_df)):
        longitudinal.append(error_x[i] * math.cos(-sync_ref_df['ref_yaw'][i]) - error_y[i] * math.sin(-sync_ref_df['ref_yaw'][i]))
        lateral.append(error_x[i] * math.sin(-sync_ref_df['ref_yaw'][i]) + error_y[i] * math.cos(-sync_ref_df['ref_yaw'][i]))
    # Longitudinal Error
    fig_longitudinal_error = plt.figure('Longitudinal_Error')
    ax_longitudinal_error = fig_longitudinal_error.add_subplot(111)
    ax_longitudinal_error.plot(time, longitudinal, marker="o", c='k', markersize=2)
    ax_longitudinal_error.set_xlabel('time[s]')
    ax_longitudinal_error.set_ylabel('error[m]')
    ax_longitudinal_error.set_title('Longitudinal Error')
    ax_longitudinal_error.grid()
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()
    
    # Lateral Error
    fig_lateral_error = plt.figure('Lateral_Error')
    ax_lateral_error = fig_lateral_error.add_subplot(111)
    ax_lateral_error.plot(time, lateral, marker="o", c='k', markersize=2)
    ax_lateral_error.set_xlabel('time[s]')
    ax_lateral_error.set_ylabel('error[m]')
    ax_lateral_error.set_title('Lateral Error')
    ax_lateral_error.grid()
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # Distance to the Nearest Reference Pose 

    # Roll Pitch Yaw Error
    # Roll Error
    fig_rpy_error = plt.figure('Roll_Pitch_Yaw_Error')
    ax_roll_error = fig_rpy_error.add_subplot(311)
    ax_roll_error.set_title('Roll Error')
    ax_roll_error.plot(time, sync_result_df['result_roll'] - sync_ref_df['ref_roll'], marker="o", c='k', markersize=2)
    ax_roll_error.set_xlabel('time[s]')
    ax_roll_error.set_ylabel('error[dgree]')
    ax_roll_error.set_aspect('equal')
    ax_roll_error.grid()

    # Pitch Error
    ax_pitch_error = fig_rpy_error.add_subplot(312)
    ax_pitch_error.set_title('Pitch Error')
    ax_pitch_error.plot(time, sync_result_df['result_pitch'] - sync_ref_df['ref_pitch'], marker="o", c='k', markersize=2)
    ax_pitch_error.set_xlabel('time[s]')
    ax_pitch_error.set_ylabel('error[dgree]')
    ax_pitch_error.set_aspect('equal')
    ax_pitch_error.grid()

    # Yaw Error
    ax_yaw_error = fig_rpy_error.add_subplot(313)
    ax_yaw_error.set_title('Yaw Error')
    ax_yaw_error.plot(time, sync_result_df['result_yaw'] - sync_ref_df['ref_yaw'], marker="o", c='k', markersize=2)
    ax_yaw_error.set_xlabel('time[s]')
    ax_yaw_error.set_ylabel('error[dgree]')
    ax_yaw_error.set_aspect('equal')
    ax_yaw_error.grid()
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # Velocity Error

    #Save Dataframe
    if save_param.save_dataframe == True:
        sync_ref_df.to_csv(output_dir + "/sync_ref_df.csv")
        sync_result_df.to_csv(output_dir + "/sync_result_df.csv")
    
    # Save Figures
    if save_param.save_figures == True:
        fig_2d_trj.savefig(output_dir + "/2d_trj." + save_param.save_extension_type)
        # fig_3d_trj.savefig(output_dir + "/3d_trj." + save_param.save_extension_type)
        fig_rpy.savefig(output_dir + "/rpy." + save_param.save_extension_type)
        fig_2d_error.savefig(output_dir + "/2d_error." + save_param.save_extension_type)
        fig_hight_error.savefig(output_dir + "/hight_error." + save_param.save_extension_type)
        fig_3d_error.savefig(output_dir + "/3d_error." + save_param.save_extension_type)
        fig_longitudinal_error.savefig(output_dir + "/longitudinal_error." + save_param.save_extension_type)
        fig_lateral_error.savefig(output_dir + "/lateral_error." + save_param.save_extension_type)
        fig_rpy_error.savefig(output_dir + "/rpy_error." + save_param.save_extension_type)

    plt.show()