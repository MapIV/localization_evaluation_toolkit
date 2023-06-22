#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=${SCRIPT_DIR%/*}

ROS_SOURCE="/opt/ros/galactic/setup.bash"
ROSBAG_PLAY=$SCRIPT_DIR"/autoware_play.py"
NDT_EVAL_PLOT=$SCRIPT_DIR"/sub_ndt_evaluation.py"
HEIGHT_EVALUATOR=$BASE_DIR"/ros2/install/eve_util/height_evaluator"
HEIGHT_PLOT=$SCRIPT_DIR"/height_plot.py"
MAP_TMP_DIR="/tmp/map"
RECORD_TOPICS="/localization/util/downsample/pointcloud /localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias /localization/pose_estimator/nearest_voxel_transformation_likelihood /localization/pose_estimator/transform_probability /localization/pose_estimator/iteration_num /localization/pose_with_covariance /tf /tf_static /localization/pose_estimator/pose_with_covariance"

function usage() {
cat <<_EOT_
Usage:
  $0 <AUTOWARE_DIR> <BAG_DIR> <PCD_FILE> <LL2_FILE> <OUTPUT_DIR>

Description:
  Evaluation scripts for eve autonomy

_EOT_

exit 1
}

# Parse option
if [ "$OPTIND" = 1 ]; then
  while getopts h OPT
  do
    case $OPT in
      h)
        usage ;;
      \?)
        echo "Undefined option $OPT"
        usage ;;
    esac
  done
else
  echo "No installed getopts-command." 1>&2
  exit 1
fi
shift $(($OPTIND - 1))

AUTOWARE_DIR=$1
BAG_DIR=$2
PCD_FILE=$3
LL2_FILE=$4
OUTPUT_DIR=$5

DATE=$(date +%Y%m%d-%H%M%S)
RECORD_DIR=$OUTPUT_DIR/"$DATE"

echo $AUTOWARE_DIR
echo $BAG_DIR
echo $PCD_FILE
echo $LL2_FILE
echo $OUTPUT_DIR

source $ROS_SOURCE
sudo ifconfig lo multicast
source $AUTOWARE_DIR"/install/setup.bash"

mkdir $MAP_TMP_DIR
cp $PCD_FILE $MAP_TMP_DIR"/pointcloud_map.pcd"
cp $LL2_FILE $MAP_TMP_DIR"/lanelet2.osm"

# ps -aux | grep ros | awk '{print $2}' | xargs kill -9
sleep 3

echo "Start Autoware!"
ros2 launch autoware_launch logging_simulator.launch.xml vehicle_model:=ymc_golfcart_proto2 sensor_model:=aip_x1 map_path:=$MAP_TMP_DIR > $OUTPUT_DIR"/autoware_log.txt" &
aw_pid=$!
sleep 10

python3 $ROSBAG_PLAY $BAG_DIR "1" &
play_pid=$!

python3 $ROSBAG_PLAY "only_subscribe" "2"
python3 $ROSBAG_PLAY "only_subscribe" "3"
sleep 10

ros2 bag record -o $RECORD_DIR $RECORD_TOPICS &
record_pid=$!

echo "Start Record!"

wait $play_pid

# kill -SIGINT $aw_pid &
kill -SIGINT $record_pid &
sleep 5
# ps -aux | grep ros | awk '{print $2}' | xargs kill -9

ros2 launch ndt_convergence_evaluation ndt_convergence_evaluation.launch.py rosbag_file_name:=$RECORD_DIR map_path:=$MAP_TMP_DIR save_dir:=$RECORD_DIR"/convergence" 
ros2 run ndt_convergence_evaluation evaluation.py --input $RECORD_DIR"/convergence" 

cd $SCRIPT_DIR
python3 $NDT_EVAL_PLOT $RECORD_DIR $RECORD_DIR 
$HEIGHT_EVALUATOR $RECORD_DIR $PCD_FILE $RECORD_DIR"/height.csv"
python3 $HEIGHT_PLOT $RECORD_DIR"/height.csv"
