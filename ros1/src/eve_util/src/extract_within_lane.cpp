#include <iostream>
#include <fstream>
#include <sstream>

#include "csv.h"

#include <lanelet2_parser/lanelet2_parser.hpp>

std::vector<std::string> split(const std::string str, char delimiter){
  std::vector<std::string> output;
  std::cout << str << std::endl;

  std::istringstream sstr(str);
  std::string tmp;

  while (std::getline(sstr, tmp, delimiter)) {
    output.push_back(tmp);
  }

  return output;

}

int main(int argc, char** argv)
{

  if(argc !=5){
    std::cerr << argv[0] << " <CSV_FILE> <OSM_FILE> <X_COLUMN_NUM> <Y_COLUMN_NUM>" << std::endl;
    exit(1);
  }

  std::string csv_name = argv[1];
  std::string osm_name = argv[2];
  int x_col = std::stoi(argv[3]);
  int y_col = std::stoi(argv[4]);

  std::cout << "Loading CSV ... " << std::endl;
  std::ifstream ifs(csv_name);
  if(ifs.fail()){
    std::cout << std::endl;
    std::cerr << "\033[31;1mError: Cannot load csv\033[m" << std::endl;
    exit(1);
  }
  std::cout << "OK." << std::endl;

  std::cout << "Loading Lanelet2 ... " << std::endl;
  LaneletParser lanelet_parser;
  if (!lanelet_parser.loadOSM(osm_name)) {
    std::cout << std::endl;
    std::cerr << "\033[31;1mError: Cannot load Lanelets\033[m" << std::endl;
    exit(1);
  }
  std::cout << "OK." << std::endl;

  std::string output = csv_name.substr(0,csv_name.length() - 4)+"_in_lane.csv";
  std::ofstream ofs(output);

  std::string str;

  size_t i=0;

  while (std::getline(ifs, str)) {
    if(i==0){
      ofs << str << std::endl;
      i++;
      continue;
    }
    auto vec = split(str,',');
    double x = std::stod(vec[x_col]);
    double y = std::stod(vec[y_col]);
    double z;
    std::cout << x_col <<"," << y_col << std::endl;
    std::cout << x <<"," << y << std::endl;
    if(lanelet_parser.isInside(x,y,z)){
      ofs << str << std::endl;
    }
    i++;
  }

  return 0;
}
