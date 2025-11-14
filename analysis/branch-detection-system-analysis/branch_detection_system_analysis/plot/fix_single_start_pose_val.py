#!/usr/bin/env python3
from branch_detection_system_analysis.fit import branch_processing as bp
from branch_detection_system_analysis.node import dummy_node
from branch_detection_system_analysis.plot import plotting_backend as pb
from final_approach_controller import curve_fitting as cf
from final_approach_controller import data_processing as dp
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go
import pprint as pp

from final_approach_controller_msgs.action import GenerateCylindricalPoses
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation

import traceback

import datetime as dt


from pathlib import Path


def get_files() -> list[str]:
    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

    no_pose_files = [
        "20250729_21-54-35",
        "20250729_21-55-06",
        "20250729_21-55-48",
        "20250729_21-57-03",
        "20250729_21-57-42",
        "20250729_21-58-24",
        "20250729_21-58-57",
        "20250729_21-59-43",
        "20250729_22-00-09",
        "20250729_22-00-41",
        "20250729_22-01-07",
        "20250729_22-01-52",
        "20250729_22-02-30",
        "20250729_22-03-07",
        "20250729_22-03-48",
        "20250729_22-04-29",
        "20250729_22-04-56",
        "20250729_22-05-28",
        "20250729_22-05-57",
        "20250729_22-06-30",
        "20250729_22-06-57",
        "20250729_22-07-35",
        "20250729_22-08-17",
        "20250729_22-08-55",
        "20250729_22-09-33",
        "20250729_22-11-21",
        "20250730_17-31-56",
        "20250730_17-32-31",
        "20250730_17-33-32",
        "20250730_17-34-08",
        "20250730_17-34-32",
        "20250730_17-34-57",
    ]

    for file_date in no_pose_files:
        files_by_datetime = pb.get_files_by_datetime_str(warehouse_path=warehouse_path, d_str=file_date)

        df_dict = pb.build_df_dict_from_files(data_dict=None, files=files_by_datetime)
        print(file_date)
        print(df_dict["trial_start_pose"])

        if df_dict["rotation_started"].empty:
            continue

        if df_dict["trial_start_pose"].empty:
            ...
            # rotation_started_ts = df_dict['rotation_started'].at[0, 'rotation_event_ts']

            # tf_df = pb.get_tf_df_at_closest_timestamp(tf_df=df_dict['tf'], tf_static_df=df_dict['tf_static'], timestamp=rotation_started_ts)
            # tf_tof_to_base = pb.get_tf_matrix_from_df(
            #     target_frame=f"amiga__base",
            #     source_frame=f"mock_pruner__tool0",
            #     tf_df=tf_df,
            # )
            # p = tf_tof_to_base[:3, 3]
            # q = Rotation.from_matrix(tf_tof_to_base[:3, :3]).as_quat()

            # df_dict['trial_start_pose'] = pd.DataFrame(columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            # df_dict['trial_start_pose'].loc[-1] = [p[0], p[1], p[2], q[0], q[1], q[2], q[3]]
            # df_dict['trial_start_pose'].index = df_dict['trial_start_pose'].index + 1

            # trial_base_dir = os.path.dirname(files_by_datetime[0])
            # trial_basename = os.path.basename(files_by_datetime[0])
            # topic_name = trial_basename.split("__")[-2]
            # start_pt_basename = trial_basename.replace(topic_name, "trial_start_pose")
            # save_path = os.path.join(trial_base_dir, start_pt_basename)

            # print(df_dict['trial_start_pose'])

            # df_dict['trial_start_pose'].to_hdf(save_path, key='data', format='fixed')

        # print(save_path)

    return


def main():
    files = get_files()

    return


if __name__ == "__main__":
    main()
