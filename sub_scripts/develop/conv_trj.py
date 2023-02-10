import os
import sys

import matplotlib.pyplot as plt
from matplotlib import cm
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# sys.path.append("../scripts")
from scripts.deprecated import adjust, read_ros2bag, yaml_param

if __name__ == "__main__":

    argv = sys.argv
    bag_path = argv[1]

    print("Read convergence_trajectory topic")
    frame = []
    read_ros2bag.read_pose_array(bag_path, frame, "/localization/pose_estimator/convergence_trajectory")

    print("Output graph")
    conv_trj_fig = plt.figure("Convergence Trajectory", figsize=(16, 9), dpi=120)
    ax_ref_conv = conv_trj_fig.add_subplot(111)
    itr = []
    result_x = []
    result_y = []
    type(len(frame[1].x))
    for i in range(len(frame)):
        ax_ref_conv.plot(frame[i].x, frame[i].y, marker="o", c="k", markersize=2, linewidth=0.5, zorder=1)
        result_x.append(frame[i].x[-1])
        result_y.append(frame[i].y[-1])
        itr.append(len(frame[i].x))

    scatter = ax_ref_conv.scatter(result_x, result_y, c=itr, cmap=cm.jet, s=12, linewidth=0, zorder=2)
    cbar = plt.colorbar(scatter, label="Iteration")

    ax_ref_conv.set_xlabel("x [m]")
    ax_ref_conv.set_ylabel("y [m]")
    ax_ref_conv.grid()
    ax_ref_conv.set_aspect("equal")
    plt.show()
