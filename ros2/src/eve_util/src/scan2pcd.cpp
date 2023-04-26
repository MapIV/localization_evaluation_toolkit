#include <cstdio>

#include <pcl/io/pcd_io.h>
#include "pcl_conversions/pcl_conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_ros/transforms.hpp>


#define POSE_TOPIC_NAME "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias"
#define POINTCLOUD_TOPIC_NAME "/localization/util/downsample/pointcloud"

int main(int argc, char * argv[]) {

  if (argc == 1)
  {
    std::cerr << std::string(argv[0]) + " <BAG_FILE> <OUTPUT_DIR>" << std::endl;
    exit(1);
  }

  std::string bag_file = argv[1];
  std::string output_dir = argv[2];

  rosbag2_cpp::Reader reader;
  reader.open(bag_file);
  std::unordered_map<std::string, std::string> topic_type_map;

  const auto &topics_and_types = reader.get_all_topics_and_types();
  for (const auto &topic : topics_and_types) {
    topic_type_map[topic.name] = topic.type;
  }

  rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped>
      pose_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2>
      scan_serialization;

  Eigen::Matrix4f transform;
  
  while (reader.has_next()) {

    auto msg = reader.read_next();
    
    if(msg->topic_name == POSE_TOPIC_NAME){
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      auto input_msg =
          std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pose_serialization.deserialize_message(&serialized_msg, input_msg.get());

      Eigen::Translation3f trans(input_msg->pose.pose.position.x, 
                                 input_msg->pose.pose.position.y, 
                                 input_msg->pose.pose.position.z);
      Eigen::Quaternionf quat(input_msg->pose.pose.orientation.w,
                              input_msg->pose.pose.orientation.x,
                              input_msg->pose.pose.orientation.y,
                              input_msg->pose.pose.orientation.z);
      Eigen::Affine3f affine = trans * quat;
      transform = affine.matrix();
    }

    if(msg->topic_name == POINTCLOUD_TOPIC_NAME){
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      auto input_msg =
          std::make_shared<sensor_msgs::msg::PointCloud2>();
      scan_serialization.deserialize_message(&serialized_msg, input_msg.get());


      sensor_msgs::msg::PointCloud2 points_msg_transform;
      pcl_ros::transformPointCloud(transform, *input_msg, points_msg_transform);
      pcl::PointCloud<pcl::PointXYZ> scan,scan_transform;
      pcl::fromROSMsg(points_msg_transform,scan_transform);

      double stamp = (double)input_msg->header.stamp.sec +
                (double)input_msg->header.stamp.nanosec * 1E-9;

      std::string filename = output_dir + "/" + std::to_string(stamp) + ".pcd";

      pcl::io::savePCDFileBinary(filename,scan_transform);
      
      continue;
    }
  }

  return 0;
}
