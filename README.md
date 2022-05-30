# localization_evaluation_toolkit -Ver.1.0
You can evaluate your localization results by comparing it to a reliable pose trajectory. The start time, end time, and period can be different for both data. The evaluation is automatically aligned with the one with the smaller number of data.

## Install
```
$ git clone https://github.com/MapIV/localization_evaluation_toolkit.git
```

## Necessary csv files
You should prepare the two types of csv files. The start time, end time, and period can be different for both data.
1. Reference csv ...Reliable pose trajectory
2. Result csv ...Localization result trajectory you'd like to evaluate

These files require at least the following elements.
- Time stamp
- Position (x,y,z)
- Rotation (Quaternion or Euler[degree or radian]) 

## How to specify the value of yaml file
Please specify the "Necessary Params" at least.
In 'Reference' and 'Result', specify the number of csv columns corresponding to the element. Depending on your csv, select the time and rotation format as True or False.
'Display' column is setting of output graphs. In 'Save' column, you can choose whether to save the graphs.

## How to run
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py [reference_csv_path] [localization_result_csv] [yaml_file] [output_folder_path]
```

## What graphs are outputed?
Sample output graphs are shown below. You can test this evaluation script easily with the following command. All graphs are outputed in full HD.
```
$ cd localization_evaluation_toolkit/scripts
$ python3 main.py ~/localization_evaluation_toolkit/sample_data/reference.csv ~/localization_evaluation_toolkit/sample_data/result_trj_ndt.csv ~/localization_evaluation_toolkit/sample_data/config/sasashima_evaluation.yaml ~/localization_evaluation_toolkit/sample_data/output_test
```

1. 2D Trajectory

![2d_trj](https://user-images.githubusercontent.com/81670028/169790998-f64bd0e2-6ace-4981-b910-1ec9974a6a9c.png)

When you zoom in on the graph, you can see the correspondence.
![2D_Trajectory_expansion](https://user-images.githubusercontent.com/81670028/169792194-a1aa8e63-68a9-4fe2-9567-9d5195e0c18b.png)

2. 2D Error

![2d_error](https://user-images.githubusercontent.com/81670028/169789941-6c06f257-6fee-4199-8769-7ddbf1afd99c.png)

3. Hight Error

![hight_error](https://user-images.githubusercontent.com/81670028/169791076-3e22a628-fa2f-4cac-a8ec-d4ab9c4e5c9e.png)

4. 3D Error

![3d_error](https://user-images.githubusercontent.com/81670028/169791037-96ecb39c-6dc9-419f-aede-32ca248a9796.png)

5. Longitudinal Error

![longitudinal_error](https://user-images.githubusercontent.com/81670028/169791156-0ccced9d-028f-4223-a7a9-3b5e980cb2e5.png)

6. Lateral Error

![lateral_error](https://user-images.githubusercontent.com/81670028/169791113-440ef4b5-e5a4-4775-93d3-5ec0017650b5.png)

7. Roll Pitch Yaw

You can choose between radian display and degree display.
![rpy](https://user-images.githubusercontent.com/81670028/169791184-8207bcf0-93aa-4b66-8665-5d049d6effba.png)

8. Roll Pitch Yaw Error

You can choose between radian display and degree display.
![rpy_error](https://user-images.githubusercontent.com/81670028/169791223-147023de-4b28-4810-a8c3-2279095e2b43.png)
