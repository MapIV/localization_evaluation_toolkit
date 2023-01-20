import rosbag
import sys
import os
import pandas as pd

# get path
argv = sys.argv
filename=os.path.normpath(os.path.join(os.getcwd(),argv[1]))
output_path = argv[2]

# read from bag file
bag = rosbag.Bag(filename)

# select topic
print '---- topic list ----'
topics = bag.get_type_and_topic_info()[1].keys()
k = 0
for val in bag.get_type_and_topic_info()[1].values():
    if val[0] == "geometry_msgs/PoseWithCovarianceStamped":
        print k, '  topic  ', topics[k]
        print '    msg    ', val[0]
        print ''
    k += 1
number = input("input topic number : ")
topic_name = topics[number]

# initialized dict
col_name = ["sec", "nsec", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"]
cov_comp = []
cov_dize = 36
for i in range(cov_dize):
    cov_name = "cov"+str(i)
    col_name.append(cov_name)
    cov_comp.append(cov_name)

pose_data_dict = {}
for i in range(len(col_name)):
    pose_data_dict[col_name[i]] = []


# msg
for topic, msg, t in bag.read_messages():
    if topic == topic_name:
        pose_data_dict["sec"].append(msg.header.stamp.secs)
        pose_data_dict["nsec"].append(msg.header.stamp.nsecs)
        pose_data_dict["x"].append(msg.pose.pose.position.x)
        pose_data_dict["y"].append(msg.pose.pose.position.y)
        pose_data_dict["z"].append(msg.pose.pose.position.z)
        pose_data_dict["q_x"].append(msg.pose.pose.orientation.x)
        pose_data_dict["q_y"].append(msg.pose.pose.orientation.y)
        pose_data_dict["q_z"].append(msg.pose.pose.orientation.z)
        pose_data_dict["q_w"].append(msg.pose.pose.orientation.w)

        for i in range(cov_dize):
            pose_data_dict[cov_comp[i]].append(msg.pose.covariance[i])
bag.close()

# to csv
df = pd.DataFrame.from_dict(pose_data_dict)
df = df[col_name]
df.to_csv(output_path + "/pose_with_covariance.csv")
print "Saved csv file"

