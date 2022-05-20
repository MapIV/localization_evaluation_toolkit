import pandas as pd
import math
import yaml
from operator import truediv
from scipy.spatial.transform import Rotation as R

def input_yaml_param(config_dir, ref_param, result_param, save_param):
    with open(config_dir, 'r') as yml:
        config = yaml.safe_load(yml)

        ref_param.separate_time_stamp = config['Reference']['separate_time_stamp']
        ref_param.stamp_column = config['Reference']['stamp_column']
        ref_param.secs_stamp_column = config['Reference']['secs_stamp_column']
        ref_param.nsecs_stamp_column = config['Reference']['nsecs_stamp_column']
        ref_param.x_column = config['Reference']['x_column']
        ref_param.y_column = config['Reference']['y_column']
        ref_param.z_column = config['Reference']['z_column']
        ref_param.use_quaternion = config['Reference']['use_quaternion']
        ref_param.ori_x_column = config['Reference']['ori_x_column']
        ref_param.ori_y_column = config['Reference']['ori_y_column']
        ref_param.ori_z_column = config['Reference']['ori_z_column']
        ref_param.ori_w_column = config['Reference']['ori_w_column']
        ref_param.use_radian = config['Reference']['use_radian']
        ref_param.roll_column = config['Reference']['roll_column']
        ref_param.pitch_column = config['Reference']['pitch_column']
        ref_param.yaw_column = config['Reference']['yaw_column']

        result_param.separate_time_stamp = config['Result']['separate_time_stamp']
        result_param.stamp_column = config['Result']['stamp_column']
        result_param.secs_stamp_column = config['Result']['secs_stamp_column']
        result_param.nsecs_stamp_column = config['Result']['nsecs_stamp_column']
        result_param.x_column = config['Result']['x_column']
        result_param.y_column = config['Result']['y_column']
        result_param.z_column = config['Result']['z_column']
        result_param.use_quaternion = config['Result']['use_quaternion']
        result_param.ori_x_column = config['Result']['ori_x_column']
        result_param.ori_y_column = config['Result']['ori_y_column']
        result_param.ori_z_column = config['Result']['ori_z_column']
        result_param.ori_w_column = config['Result']['ori_w_column']
        result_param.use_radian = config['Result']['use_radian']
        result_param.roll_column = config['Result']['roll_column']
        result_param.pitch_column = config['Result']['pitch_column']
        result_param.yaw_column = config['Result']['yaw_column']

        save_param.font = config['font']
        save_param.save_figures = config['save_figures']
        save_param.save_extension_type = config['save_extension_type']
        save_param.save_dataframe = config['save_dataframe']

def unit_adjust(ref_param, result_param, ref_df_temp, result_df_temp, ref_df, result_df):
    # Time
    # reference
    if ref_param.separate_time_stamp == True:
        ref_df['ref_time'] = ref_df_temp.iloc[:,ref_param.secs_stamp_column] + ref_df_temp.iloc[:,ref_param.nsecs_stamp_column] / 10**9
    else:
        ref_df['ref_time'] = ref_df_temp.iloc[:,ref_param.stamp_column]

    # result
    if result_param.separate_time_stamp == True:
        result_df['result_time'] = result_df_temp.iloc[:,result_param.secs_stamp_column] + result_df_temp.iloc[:,result_param.nsecs_stamp_column] / 10**9
    else:
        result_df['result_time'] = result_df_temp.iloc[:,result_param.stamp_column]
    
    # Position
    # reference
    ref_df['ref_x'] = ref_df_temp.iloc[:,ref_param.x_column]
    ref_df['ref_y'] = ref_df_temp.iloc[:,ref_param.y_column]
    ref_df['ref_z'] = ref_df_temp.iloc[:,ref_param.z_column]

    # result
    result_df['result_x'] = result_df_temp.iloc[:,result_param.x_column]
    result_df['result_y'] = result_df_temp.iloc[:,result_param.y_column]
    result_df['result_z'] = result_df_temp.iloc[:,result_param.z_column]

    # Rotation
    # reference
    if ref_param.use_quaternion == True:
        for i in range(len(ref_df_temp)):
            ref_q_temp = [ref_df_temp.iloc[i,ref_param.ori_x_column],ref_df_temp.iloc[i,ref_param.ori_y_column],ref_df_temp.iloc[i,ref_param.ori_z_column],ref_df_temp.iloc[i,ref_param.ori_w_column]]
            ref_e_temp = R.from_quat([ref_q_temp[0], ref_q_temp[1], ref_q_temp[2], ref_q_temp[3]])
            ref_df.at[i,'ref_roll'] = ref_e_temp.as_euler('xyz', degrees=False)[0]
            ref_df.at[i,'ref_pitch'] = ref_e_temp.as_euler('xyz', degrees=False)[1]
            ref_df.at[i,'ref_yaw'] = ref_e_temp.as_euler('xyz', degrees=False)[2]
    elif ref_param.use_quaternion == False and ref_param.use_radian == False:
        ref_df['ref_roll'] = ref_df_temp.iloc[:,ref_param.roll_column]
        ref_df['ref_pitch'] = ref_df_temp.iloc[:,ref_param.pitch_column]
        ref_df['ref_yaw'] = ref_df_temp.iloc[:,ref_param.yaw_column]
    elif ref_param.use_quaternion == False and ref_param.use_radian == True:
        ref_df['ref_roll'] = ref_df_temp.iloc[:,ref_param.roll_column] * math.pi / 180
        ref_df['ref_pitch'] = ref_df_temp.iloc[:,ref_param.pitch_column] * math.pi / 180
        ref_df['ref_yaw'] = ref_df_temp.iloc[:,ref_param.yaw_column] * math.pi / 180

    # result
    if result_param.use_quaternion == True:
        for i in range(len(result_df_temp)):
            result_q_temp = [result_df_temp.iloc[i,result_param.ori_x_column],result_df_temp.iloc[i,result_param.ori_y_column],result_df_temp.iloc[i,result_param.ori_z_column],result_df_temp.iloc[i,result_param.ori_w_column]]
            result_e_temp = R.from_quat([result_q_temp[0], result_q_temp[1], result_q_temp[2], result_q_temp[3]])
            result_df.at[i,'result_roll'] = result_e_temp.as_euler('zyx', degrees=False)[2]
            result_df.at[i,'result_pitch'] = result_e_temp.as_euler('zyx', degrees=False)[1]
            result_df.at[i,'result_yaw'] = result_e_temp.as_euler('zyx', degrees=False)[0]
    elif result_param.use_quaternion == False and result_param.use_radian == False:
        result_df['result_roll'] = result_df_temp.iloc[:,result_param.roll_column]
        result_df['result_pitch'] = result_df_temp.iloc[:,result_param.pitch_column]
        result_df['result_yaw'] = result_df_temp.iloc[:,result_param.yaw_column]
    elif result_param.use_quaternion == False and result_param.use_radian == True:
        result_df['result_roll'] = result_df_temp.iloc[:,result_param.roll_column] * math.pi / 180
        result_df['result_pitch'] = result_df_temp.iloc[:,result_param.pitch_column] * math.pi / 180
        result_df['result_yaw'] = result_df_temp.iloc[:,result_param.yaw_column] * math.pi / 180

def adjust_start_time(ref_df,result_df):
    if ref_df.at[0,'ref_time'] < result_df.at[0,'result_time'] and ref_df['ref_time'].iloc[-1] > result_df.at[0,'result_time']:
       search_ref_start = abs(ref_df['ref_time'] - result_df.at[0,'result_time'])
       ref_start_time_index = search_ref_start.idxmin()
       ref_df.drop(range(0,ref_start_time_index), inplace=True)
       ref_df.reset_index(inplace=True,drop=True)
    elif ref_df.at[0,'ref_time'] > result_df.at[0,'result_time'] and ref_df.at[0,'ref_time'] < result_df['result_time'].iloc[-1]:
       search_result_start = abs(result_df['result_time'] - ref_df.at[0,'ref_time'])
       print("search_result_start")
       print(search_result_start)
       result_start_time_index = search_result_start.idxmin()
       print("result_start_time_index")
       print(result_start_time_index)
       result_df.drop(range(0,result_start_time_index), inplace=True)
       result_df.reset_index(inplace=True,drop=True)
       print(result_df)
    elif ref_df.at[0,'ref_time'] == result_df.at[0,'result_time']:
        pass
    else:
        print('[ERROR]: Reference time stamp and Result time stamp do not overlap')
        return -1

def adjust_end_time(ref_df,result_df):
    if ref_df['ref_time'].iloc[-1] > result_df['result_time'].iloc[-1]:
       search_ref_end = abs(ref_df['ref_time'] - result_df['result_time'].iloc[-1])
       ref_end_time_index = search_ref_end.idxmin()    
       ref_df.drop(range(ref_end_time_index+1,len(ref_df.index)), inplace=True)
       ref_df.reset_index(inplace=True,drop=True)   
    elif ref_df['ref_time'].iloc[-1] < result_df['result_time'].iloc[-1]:
       search_result_end = abs(result_df['result_time'] - ref_df['ref_time'].iloc[-1])
       result_end_time_index = search_result_end.idxmin()   
       result_df.drop(range(result_end_time_index+1,len(result_df.index)), inplace=True)
       result_df.reset_index(inplace=True,drop=True)

def sync_time(ref_df, result_df):   
    if len(ref_df.index) >= len(result_df.index):
        sync_ref_df = pd.DataFrame()
        for i in range(0,len(result_df)):
            search_sync_ref_time = abs(ref_df['ref_time'] - result_df.at[i,'result_time'])
            sync_ref_time = search_sync_ref_time.idxmin()
            sync_ref_df = sync_ref_df.append(ref_df.iloc[sync_ref_time,:], ignore_index = True)
        return sync_ref_df, result_df
    else:
        sync_result_df = pd.DataFrame()
        for i in range(0,len(ref_df)):
            search_sync_result_time = abs(result_df['result_time'] - ref_df.at[i,'ref_time'])
            sync_result_time = search_sync_result_time.idxmin()
            sync_result_df = sync_result_df.append(result_df.iloc[sync_result_time,:], ignore_index = True)
        return ref_df, sync_result_df