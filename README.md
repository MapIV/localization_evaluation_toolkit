# localization_evaluation_toolkit
 

## Install
```
$ git clone https://github.com/MapIV/localization_evaluation_toolkit.git
```

## Necessary csv files
You should prepare the two types of csv files. It's okay if the start time, end time, and cycle are different.
1. Reference csv
2. Localization result csv

These files require at least the following elements.
- Time stamp
- Position (x,y,z)
- Rotation (Quaternion or Euler[degree or radian]) 

## How to specify the value of yaml file
Please specify the "Necessary Params" at least.

## How to run
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py [reference_csv_path] [localization_result_csv] [yaml_file] [output_folder_path]
```