# Elevation Map Checker

(Updated 2022/11/04)

## Installation

```
mkdir -p ll2_ws/src && cd ll2_ws/src
git clone git@github.com:MapIV/elevation_map_checker.git
git clone git@github.com:MapIV/lanelet2_parser.git
git clone git@github.com:MapIV/llh_converter.git

cd ../
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

catkin_make -DCMAKE_BUILD_TYPE=Release
```

## How to Run

```
roscore (in another terminal)
./src/elevation_map_checker/scripts/elevation_map.sh <PCD_FILE> <OSM_FILE>
```
