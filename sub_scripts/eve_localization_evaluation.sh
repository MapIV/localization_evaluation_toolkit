#!/bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
BASE_DIR=${SCRIPT_DIR%/*}
NDT_EVAL_PLOT=$SCRIPT_DIR"/sub_ndt_evaluation.py"
HEIGHT_EVALUATOR=$BASE_DIR"/ros2/install/eve_util/height_evaluator"
HEIGHT_PLOT=$SCRIPT_DIR"/height_plot.py"

function usage() {
cat <<_EOT_
Usage:
  $0 <BAG_FILE> <PCD_FILE> <OUTPUT_DIR>

Description:
  Evaluation scripts for eve

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


BAG_FILE=$1
PCD_FILE=$2
OUTPUT_DIR=$3

cd $SCRIPT_DIR
python3 $NDT_EVAL_PLOT $BAG_FILE $OUTPUT_DIR 
$HEIGHT_EVALUATOR $BAG_FILE $PCD_FILE $OUTPUT_DIR"/height.csv"
python3 $HEIGHT_PLOT $OUTPUT_DIR"/height.csv"
