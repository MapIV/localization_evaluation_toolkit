#include "Eigen/Eigenvalues"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tf2/utils.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
  if(argc !=4){
    std::cerr << argv[0] << " <BAG_FILE> <PCD_FILE> <OUTPUT_CSF>" << std::endl;
    exit(1);
  }

  std::string bag_file = argv[1];
  std::string pcd_file = argv[2];
  std::string output_file = argv[3];

  rosbag2_cpp::Reader reader;
  reader.open(bag_file);
  std::unordered_map<std::string, std::string> topic_type_map;
  const auto &topics_and_types = reader.get_all_topics_and_types();
  for (const auto &topic : topics_and_types) {
    topic_type_map[topic.name] = topic.type;
  }
  rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped>
      pose_serialization;

  // Loading PCD
  std::cout << "Reading PCD file..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(pcd_file, *input_pcd) == -1)
  {
    std::cerr << "Error: Cannot load PCD: " + pcd_file << std::endl;
    exit(1);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(input_pcd);
  int K=1;
  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  std::ofstream ofs(output_file);
  ofs << "timestamp,x,y,z,qx,qy,qz,qw,score" << std::endl;

  while (reader.has_next()) {

    auto msg = reader.read_next();
    
    if(msg->topic_name != "/localization/pose_with_covariance"){
      continue;
    }

    const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

    auto input_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_serialization.deserialize_message(&serialized_msg, input_msg.get());

    pcl::PointXYZ p;
    p.x = input_msg->pose.pose.position.x;
    p.y = input_msg->pose.pose.position.y;
    p.z = input_msg->pose.pose.position.z;

    double sum = 0;
    if (tree.nearestKSearch(p, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
    {
      for (std::size_t j = 0; j < pointIdxKNNSearch.size(); j++)
      {
        sum += std::sqrt(pointKNNSquaredDistance[j]);
      }
      sum /= (double)pointIdxKNNSearch.size();
    }

    double stamp = (double)input_msg->header.stamp.sec +
                   (double)input_msg->header.stamp.nanosec * 1E-9;

    ofs << std::setprecision(15) << stamp << ","
        << input_msg->pose.pose.position.x << ","
        << input_msg->pose.pose.position.y << ","
        << input_msg->pose.pose.position.z << ","
        << input_msg->pose.pose.orientation.x << ","
        << input_msg->pose.pose.orientation.y << ","
        << input_msg->pose.pose.orientation.z << ","
        << input_msg->pose.pose.orientation.w << "," 
        << sum << std::endl;
  }

  ofs.close();
  
  std::cout << "Complete!!" << std::endl;

  return 0;
}
