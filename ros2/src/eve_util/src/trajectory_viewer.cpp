#include <cstdio>

#include <pcl/io/pcd_io.h>
#include "pcl_conversions/pcl_conversions.h"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "std_msgs/msg/string.hpp"

void pose2path(const std::string& bag_file, nav_msgs::msg::Path& path){

  rosbag2_cpp::Reader reader;
  reader.open(bag_file);
  std::unordered_map<std::string, std::string> topic_type_map;

  const auto &topics_and_types = reader.get_all_topics_and_types();
  for (const auto &topic : topics_and_types) {
    topic_type_map[topic.name] = topic.type;
  }

  rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped>
      pose_serialization;
  
  while (reader.has_next()) {

    auto msg = reader.read_next();
    
    if(msg->topic_name != "/localization/pose_with_covariance"){
      continue;
    }

    const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

    auto input_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_serialization.deserialize_message(&serialized_msg, input_msg.get());

    geometry_msgs::msg::PoseStamped pose;
    pose.header = input_msg->header;
    pose.pose = input_msg->pose.pose;
    path.poses.push_back(pose);
  }

  path.header = path.poses.back().header;

}


void pcd2msg(const std::string& pcd_file, sensor_msgs::msg::PointCloud2& pc2)
{
  // Loading PCD
  std::cout << "Reading PCD file..." << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcd(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile(pcd_file, *input_pcd) == -1)
  {
    std::cerr << "Error: Cannot load PCD: " + pcd_file << std::endl;
    exit(1);
  }
  pcl::toROSMsg(*input_pcd,pc2);

}


void pose2tf(const geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::TransformStamped& transform)
{
  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation.x = pose.pose.orientation.x;
  transform.transform.rotation.y = pose.pose.orientation.y;
  transform.transform.rotation.z = pose.pose.orientation.z;
  transform.transform.rotation.w = pose.pose.orientation.w;
  transform.header.stamp = pose.header.stamp;
  transform.header.frame_id = pose.header.frame_id;
  transform.child_frame_id = "viewer";

}


int main(int argc, char * argv[]) {

  if (argc == 1)
  {
    std::cerr << std::string(argv[0]) + " <BAG_FILE> <PCD_FILE>" << std::endl;
    exit(1);
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("publisher_node");

  std::string bag_file = argv[1];
  std::string pcd_file = argv[2];

  auto publisher_path = node->create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS{1}.transient_local());
  auto publisher_map = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_map", rclcpp::QoS(1).transient_local());
  tf2_ros::StaticTransformBroadcaster tf_broascaster_(node);

  nav_msgs::msg::Path path;
  pose2path(bag_file,path);

  sensor_msgs::msg::PointCloud2 pc2;
  pcd2msg(pcd_file,pc2);
  pc2.header = path.header;

  geometry_msgs::msg::TransformStamped transform;
  pose2tf(path.poses[0],transform);

  publisher_path->publish(path);
  publisher_map->publish(pc2);
  tf_broascaster_.sendTransform(transform);
  std::cout << "Publish" << std::endl;
 
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
