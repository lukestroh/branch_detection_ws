#!/usr/bin/env python3
import glob
import itertools
import numpy as np
import os
from pathlib import Path
import pandas as pd
import plotly.graph_objects as go
import plotly.subplots


from branch_detection_system_analysis.bag_reader.bag_reader import BagReader
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates

from geometry_msgs.msg import WrenchStamped
from lifecycle_msgs.msg import TransitionEvent, State
from tof_msgs.msg import TofStamped
from sensor_msgs.msg import JointState
from vl53l4cd_msgs.msg import Vl53l4cdStamped

import sys
import pprint as pp


import rclpy.logging

logger = rclpy.logging.get_logger("plot")


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))


def get_dbs(city: str, farm: str, date: str = "") -> list[str]:
    bags_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection")
    # print(bags_path)
    if date != "":
        files = glob.glob(bags_path + f"/**/*{city}_{farm}*__{date}*.db3.zstd")
    else:
        files = glob.glob(bags_path + f"/**/*{city}_{farm}*.db3.zstd")
    return files


def get_bag_reader(db: str):
    return BagReader(bag_file=db)


def get_tof_raw_data(br: BagReader) -> tuple:
    data_tof_raw = list(br.query(topic_name="/microROS/vl53l4cd/data"))
    tof_raw_data: list[Vl53l4cdStamped] = [d[1] for d in data_tof_raw]
    tof0_raw_ts, tof0_raw_data = zip(
        *[
            (tof.header.stamp.sec + tof.header.stamp.nanosec * 1e-9, tof.distance / 1000)
            for tof in tof_raw_data
            if tof.dev_id == 0
        ]
    )

    tof1_raw_ts, tof1_raw_data = zip(
        *[
            (tof.header.stamp.sec + tof.header.stamp.nanosec * 1e-9, tof.distance / 1000)
            for tof in tof_raw_data
            if tof.dev_id == 1
        ]
    )

    return (tof0_raw_ts, tof0_raw_data, tof1_raw_ts, tof1_raw_data)


def get_tof_filtered_data(br: BagReader):
    data_tof_filtered = list(br.query(topic_name="/vl53l4cd/filtered"))
    tof_filtered_data: list[TofStamped] = [d[1] for d in data_tof_filtered]
    tof0_filtered_ts, tof0_filtered_data = zip(
        *[
            (tof.header.stamp.sec + tof.header.stamp.nanosec * 1e-9, tof.data[0])
            for tof in tof_filtered_data
            if tof.dev_id == 0
        ]
    )
    # tof0_filtered_data = [tof.data[0] for tof in tof_filtered_data if tof.dev_id == 0]

    tof1_filtered_ts, tof1_filtered_data = zip(
        *[
            (tof.header.stamp.sec + tof.header.stamp.nanosec * 1e-9, tof.data[0])
            for tof in tof_filtered_data
            if tof.dev_id == 1
        ]
    )
    # tof1_filtered_data = [tof.data[0] for tof in tof_filtered_data if tof.dev_id == 1]

    return (tof0_filtered_ts, tof0_filtered_data, tof1_filtered_ts, tof1_filtered_data)


def get_wrench_data(br: BagReader):

    # FT-wrench data
    data_ft_wrench = list(br.query(topic_name="/force_torque_sensor_broadcaster/wrench"))
    wrench_data: list[WrenchStamped] = [d[1] for d in data_ft_wrench]
    wrench_data_ts, wrench_data_fx, wrench_data_fy, wrench_data_fz, wrench_data_tx, wrench_data_ty, wrench_data_tz = (
        zip(
            *map(
                lambda w: [
                    w.header.stamp.sec + w.header.stamp.nanosec * 1e-9,
                    w.wrench.force.x,
                    w.wrench.force.y,
                    w.wrench.force.z,
                    w.wrench.torque.x,
                    w.wrench.torque.y,
                    w.wrench.torque.z,
                ],
                wrench_data,
            )
        )
    )
    return (
        wrench_data_ts,
        wrench_data_fx,
        wrench_data_fy,
        wrench_data_fz,
        wrench_data_tx,
        wrench_data_ty,
        wrench_data_tz,
    )


def get_joint_angle_data(br: BagReader):
    joint_angles = list(br.query(topic_name="/joint_states"))
    joint_angle_data: list[JointState] = [d[1] for d in joint_angles]

    joint_angles_ts, joint_angles_pos = zip(
        *map(lambda ja: [ja.header.stamp.sec + ja.header.stamp.nanosec * 1e-9, ja.position], joint_angle_data)
    )

    return (joint_angles_ts, joint_angles_pos)


def get_forward_position_controller_events(br: BagReader):
    fpc_transition_events = list(br.query("/forward_position_controller/transition_event"))
    fpc_transition_events_ts, fpc_transition_events_data = zip(
        *map(lambda fpce: [fpce[0] * 1e-9, fpce[1]], fpc_transition_events)
    )

    fpc_transition_start_state, fpc_transition_goal_state = zip(
        *map(lambda fpc_t: [fpc_t.start_state.id, fpc_t.goal_state.id], fpc_transition_events_data)
    )

    return (fpc_transition_events_ts, fpc_transition_start_state, fpc_transition_goal_state)


def get_scaled_joint_trajectory_controller_events(br: BagReader):
    sjtc_transition_events = list(br.query("/scaled_joint_trajectory_controller/transition_event"))
    sjtc_transition_events_ts, sjtc_transition_events_data = zip(
        *map(lambda sjtce: [sjtce[0] * 1e-9, sjtce[1]], sjtc_transition_events)
    )

    sjtc_transition_start_state, sjtc_transition_goal_state = zip(
        *map(lambda sjtc_t: [sjtc_t.start_state.id, sjtc_t.goal_state.id], sjtc_transition_events_data)
    )

    return (sjtc_transition_events_ts, sjtc_transition_start_state, sjtc_transition_goal_state)


def get_df_from_bag_reader(br: BagReader) -> pd.DataFrame:
    br.topics.sort()
    pp.pprint(br.topics)

    tof0_filtered_ts, tof0_filtered_data, tof1_filtered_ts, tof1_filtered_data = get_tof_filtered_data(br=br)

    tof0_raw_ts, tof0_raw_data, tof1_raw_ts, tof1_raw_data = get_tof_raw_data(br=br)

    wrench_data_ts, wrench_data_fx, wrench_data_fy, wrench_data_fz, wrench_data_tx, wrench_data_ty, wrench_data_tz = (
        get_wrench_data(br=br)
    )

    joint_angle_ts, joint_angle_pos = get_joint_angle_data(br=br)

    fpc_transition_events_ts, fpc_transition_start_state, fpc_transition_goal_state = (
        get_forward_position_controller_events(br=br)
    )

    sjtc_transition_events_ts, sjtc_transition_start_state, sjtc_transition_goal_state = (
        get_scaled_joint_trajectory_controller_events(br=br)
    )

    # Get the minimum time, usually from the TOF values since microROS starts quickly
    try:
        if tof0_raw_ts[0] <= tof1_raw_ts[0]:
            time_begin = tof0_raw_ts[0]
        else:
            time_begin = tof1_raw_ts[0]
    except Exception as e:
        logger.warn(f"{e}")
        return

    # Create dataframe
    df = pd.DataFrame(
        data=list(
            itertools.zip_longest(
                np.array(tof0_raw_ts) - time_begin,
                tof0_raw_data,
                np.array(tof1_raw_ts) - time_begin,
                tof1_raw_data,
                np.array(tof0_filtered_ts) - time_begin,
                tof0_filtered_data,
                np.array(tof1_filtered_ts) - time_begin,
                tof1_filtered_data,
                np.array(wrench_data_ts) - time_begin,
                wrench_data_fx,
                wrench_data_fy,
                wrench_data_fz,
                wrench_data_tx,
                wrench_data_ty,
                wrench_data_tz,
                joint_angle_ts,
                joint_angle_pos,
                np.array(fpc_transition_events_ts) - time_begin,
                fpc_transition_start_state,
                fpc_transition_goal_state,
                np.array(sjtc_transition_events_ts) - time_begin,
                sjtc_transition_start_state,
                sjtc_transition_goal_state,
                fillvalue=np.nan,
            )
        ),
        columns=[
            "tof0_raw_ts",
            "tof0_raw_data",
            "tof1_raw_ts",
            "tof1_raw_data",
            "tof0_filtered_ts",
            "tof0_filtered_data",
            "tof1_filtered_ts",
            "tof1_filtered_data",
            "wrench_ts",
            "wrench_fx",
            "wrench_fy",
            "wrench_fz",
            "wrench_tx",
            "wrench_ty",
            "wrench_tz",
            "joint_angle_ts",
            "joint_angle_pos",
            "fpc_transition_events_ts",
            "fpc_transition_start_state",
            "fpc_transition_goal_state",
            "sjtc_transition_events_ts",
            "sjtc_transition_start_state",
            "sjtc_transition_goal_state",
        ],
    )

    print(df.head(10))

    return df


def filter_trials_for_transition_states(df: pd.DataFrame):
    # idxs_event_fpc_deactivate = df.index[df['fpc_transition_start_state'].fillna(-1).astype(int)==TransitionStates.TRANSITION_STATE_DEACTIVATING.value].to_list()

    # idxs_event_sjtc_deactivate = df.index[df['sjtc_transition_start_state'].fillna(-1).astype(int)==TransitionStates.TRANSITION_STATE_DEACTIVATING.value].to_list()
    # print(idxs_event_fpc_deactivate)
    # print(idxs_event_sjtc_deactivate)

    df_transition_events = df.loc[
        (
            df["fpc_transition_start_state"].fillna(-1).astype(int)
            == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
        )
        | (
            df["sjtc_transition_start_state"].fillna(-1).astype(int)
            == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
        ),
        [
            "fpc_transition_events_ts",
            "fpc_transition_start_state",
            "fpc_transition_goal_state",
            "sjtc_transition_events_ts",
            "sjtc_transition_start_state",
            "sjtc_transition_goal_state",
        ],
    ]

    return df_transition_events


def get_bin_mask(big_df: pd.DataFrame, start_time: float, end_time: float, time_str: str):
    return (big_df[time_str] >= start_time) & (big_df[time_str] <= end_time)


def slice_dfs_by_transition_event(big_df: pd.DataFrame, transition_event_df: pd.DataFrame) -> list[pd.DataFrame]:
    # Grab the ToF, wrench, etc. data between transition events
    trial_dfs = []

    logger.info("Slicing dataframes by data channel")

    df_fpc_transitions = transition_event_df.loc[
        transition_event_df["fpc_transition_start_state"] == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
    ].reset_index()

    # Get start and end timestamps from transition events
    i = 0
    while True:
        try:
            start_time = df_fpc_transitions.at[i * 2, "fpc_transition_events_ts"]
            end_time = df_fpc_transitions.at[i * 2 + 2, "fpc_transition_events_ts"]

            df_tof0_raw_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof0_raw_ts"), ["tof0_raw_ts", "tof0_raw_data"]
            ]
            df_tof1_raw_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof1_raw_ts"), ["tof1_raw_ts", "tof1_raw_data"]
            ]
            df_tof0_filtered_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof0_filtered_ts"),
                ["tof0_filtered_ts", "tof0_filtered_data"],
            ]
            df_tof1_filtered_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "tof1_filtered_ts"),
                ["tof1_filtered_ts", "tof1_filtered_data"],
            ]
            df_wrench_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "wrench_ts"),
                ["wrench_ts", "wrench_fx", "wrench_fy", "wrench_fz", "wrench_tx", "wrench_ty", "wrench_tz"],
            ]
            df_joint_angles_trial = big_df.loc[
                get_bin_mask(big_df, start_time, end_time, "joint_angle_ts"),
                ["joint_angle_ts", "joint_angle_pos"],
            ]

            trial_dfs.append(
                {
                    "trial_num": i,
                    "tof0_raw": df_tof0_raw_trial,
                    "tof1_raw": df_tof1_raw_trial,
                    "tof0_filtered": df_tof0_filtered_trial,
                    "tof1_filtered": df_tof1_filtered_trial,
                    "wrench": df_wrench_trial,
                    "joint_angles": df_joint_angles_trial,
                }
            )

            # print(trial_dfs)

            # break

            i += 1
        except (KeyError, ValueError):
            break

    return trial_dfs


def warehouse_trial_dfs(trial_dfs: list[dict[str, pd.DataFrame]], filename: str):
    name = Path(Path(filename).stem).stem
    trial_path = os.path.join(warehouse_path, name)
    os.mkdir(trial_path)
    for trial in trial_dfs:
        for topic_name, df in trial.items():
            if topic_name.endswith("trial_num"):
                continue
            else:
                df.to_hdf(
                    os.path.join(trial_path, name + f"__{topic_name}__{str(trial['trial_num']).zfill(3)}.h5"),
                    key=name,
                )

    return


def main():
    dbs = get_dbs(city="prosser", farm="roza", date="20250221")
    for db in dbs:
        br = get_bag_reader(db=db)
        df = get_df_from_bag_reader(br=br)
        br.cleanup()

        df_transition_events = filter_trials_for_transition_states(df=df)

        trial_dfs = slice_dfs_by_transition_event(big_df=df, transition_event_df=df_transition_events)

        warehouse_trial_dfs(trial_dfs=trial_dfs, filename=db)

        # fig = plot_data(df=trial_dfs[0], filename=db)

        # fig = plot_dict_data(data=trial_dfs[0], filename=db)

        # fig.show()

        # res = input("Yes or no: ")
        # if res == "yes":
        # fig = plot_data(df=df, filename=db)
        # fig = plot_transition_events(df=df_transition_events, fig=fig)
        # fig.show()
        #     break
        # else:
        #     break

        # break
    return


if __name__ == "__main__":
    main()
