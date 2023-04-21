#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=${SCRIPT_DIR%/*}
VIEWER=${BASE_DIR}"/ros2/install/eve_util/trajectory_viewer"

function usage() {
cat <<_EOT_
Usage:
  $0 <BAG_FILE> <PCD_FILE>

Description:
  Visualize Trajectory on Map

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

rviz2 -d ${BASE_DIR}"/ros2/src/eve_util/rviz/trajectory_publisher.rviz" &

echo $1
echo $2

$VIEWER $1 $2
