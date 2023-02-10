#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

struct TrajectoryPoint
{
  double stamp;
  double x, y, z;
  double roll, pitch, yaw;
};

std::vector<std::string> split(std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;

  while (std::getline(stream, field, delimiter))
  {
    result.push_back(field);
  }
  return result;
}

int main(int argc, char** argv)
{
  std::string csv_name = argv[1];
  std::string output_name = argv[2];
  std::string pcd_name = argv[3];
  if(argc !=4){
    std::cerr << "./build/height_evaluator <input_csv> <output_csv> <pcd>" << std::endl;
    exit(1);
  }
  
  // Read pose csv
  std::cout << "Reading pose CSV file..." << std::endl;
  int cnt = 0;
  std::string line;

  std::vector<TrajectoryPoint> traj_vec;

  std::ifstream ifs(csv_name);
  if (!ifs.is_open())
  {
    std::cerr << "\033[31;1mError: Cannot open csv: " << csv_name << "\033[m" << std::endl;
    exit(2);
  }


  // Skip header line
  std::getline(ifs, line);

  while (std::getline(ifs, line))
  {
    TrajectoryPoint item;
    cnt++;

    std::vector<std::string> str_vec = split(line, ',');

    item.stamp = std::stod(str_vec.at(0)) + std::stod(str_vec.at(1))/1000000000;
    item.x = std::stod(str_vec.at(3));
    item.y = std::stod(str_vec.at(4));
    item.z = std::stod(str_vec.at(5));

    traj_vec.push_back(item);
  }

  // Loading PCD
  std::cout << "Reading PCD file..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(pcd_name, *input_pcd);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(input_pcd);

  int K = 1;
  
  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  std::ofstream ofs(output_name);
  ofs << "sequence,timestamp,relative_stamp,x,y,z,score" << std::endl;
  for (int i = 0; i < traj_vec.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = traj_vec[i].x;
    p.y = traj_vec[i].y;
    p.z = traj_vec[i].z;

    double sum = 0;
    if (tree.nearestKSearch(p, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
    {
      for (std::size_t j = 0; j < pointIdxKNNSearch.size(); j++)
      {
        sum += std::sqrt(pointKNNSquaredDistance[j]);
      }
      sum /= (double)pointIdxKNNSearch.size();
    }

    ofs << i << ","
        << std::setprecision(15) << traj_vec[i].stamp << ","
        << std::setprecision(15) << traj_vec[i].stamp -traj_vec[0].stamp << ","
        << std::setprecision(15) << p.x << ","
        << std::setprecision(15) << p.y << ","
        << std::setprecision(15) << p.z << ","
        << sum << std::endl;
  }

  ofs.close();
  
  std::cout << "Complete!!" << std::endl;

  return 0;
}
