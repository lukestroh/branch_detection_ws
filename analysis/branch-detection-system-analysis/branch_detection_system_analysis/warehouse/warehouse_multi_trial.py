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
from tf2_msgs.msg import TFMessage
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


def get_bag_reader(db: str) -> BagReader:
    return BagReader(bag_file=db)


def get_tof_raw_data(br: BagReader) -> tuple[dict]:
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

    return (
        {
            "tof0_raw_ts": tof0_raw_ts,
            "tof0_raw_data": tof0_raw_data,
        },
        {
            "tof1_raw_ts": tof1_raw_ts,
            "tof1_raw_data": tof1_raw_data,
        },
    )


def get_tof_filtered_data(br: BagReader) -> tuple[dict]:
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

    return (
        {
            "tof0_filtered_ts": tof0_filtered_ts,
            "tof0_filtered_data": tof0_filtered_data,
        },
        {
            "tof1_filtered_ts": tof1_filtered_ts,
            "tof1_filtered_data": tof1_filtered_data,
        },
    )


def get_wrench_data(br: BagReader) -> dict:

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
    return {
        "wrench_ts": wrench_data_ts,
        "wrench_fx": wrench_data_fx,
        "wrench_fy": wrench_data_fy,
        "wrench_fz": wrench_data_fz,
        "wrench_tx": wrench_data_tx,
        "wrench_ty": wrench_data_ty,
        "wrench_tz": wrench_data_tz,
    }


def get_joint_states_data(br: BagReader) -> dict:
    joint_states = list(br.query(topic_name="/joint_states"))
    joint_states_data: list[JointState] = [d[1] for d in joint_states]

    joint_states_ts, joint_states_pos = zip(
        *map(lambda ja: [ja.header.stamp.sec + ja.header.stamp.nanosec * 1e-9, ja.position], joint_states_data)
    )

    return {"joint_states_ts": joint_states_ts, "joint_states_pos": np.array(joint_states_pos)}


def get_tf_data(br: BagReader, static=False) -> dict:
    if static:
        topic_name = "tf_static"
    else:
        topic_name = "tf"
    tf_msgs = list(br.query(topic_name=f"/{topic_name}"))
    tf_data: list[TFMessage] = [d[1] for d in tf_msgs]

    # Unpack all transforms from the list of transforms in each TFMessage
    all_transforms = list(itertools.chain.from_iterable(map(lambda transform: transform.transforms, tf_data)))
    tf_ts, tf_frame_id, tf_child_frame_id, tf_t_x, tf_t_y, tf_t_z, tf_r_x, tf_r_y, tf_r_z, tf_r_w = zip(
        *map(
            lambda tf: [
                tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9,
                tf.header.frame_id,
                tf.child_frame_id,
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ],
            all_transforms,
        )
    )

    return {
        f"{topic_name}_ts": tf_ts,
        f"{topic_name}_frame_id": tf_frame_id,
        f"{topic_name}_child_frame_id": tf_child_frame_id,
        f"{topic_name}_t_x": tf_t_x,
        f"{topic_name}_t_y": tf_t_y,
        f"{topic_name}_t_z": tf_t_z,
        f"{topic_name}_r_x": tf_r_x,
        f"{topic_name}_r_y": tf_r_y,
        f"{topic_name}_r_z": tf_r_z,
        f"{topic_name}_r_w": tf_r_w,
    }


def get_controller_events(br: BagReader, controller_name: str) -> dict:
    controller_events = list(br.query(topic_name=f"/{controller_name}/transition_event"))
    controller_transition_events_ts, controller_events_data = zip(
        *map(lambda ctrlr: [ctrlr[0] * 1e-9, ctrlr[1]], controller_events)
    )

    controller_transition_start_state, controller_transition_goal_state = zip(
        *map(lambda ctrlr_t: [ctrlr_t.start_state.id, ctrlr_t.goal_state.id], controller_events_data)
    )

    return {
        "controller_transition_events_ts": controller_transition_events_ts,
        "controller_transition_start_state": controller_transition_start_state,
        "controller_transition_goal_state": controller_transition_goal_state,
    }


def create_df_from_data_dict(data: dict) -> pd.DataFrame:
    df = pd.DataFrame(data=list(itertools.zip_longest(*data.values(), fillvalue=np.nan)), columns=list(data.keys()))
    return df


def get_dfs_from_bag_reader(br: BagReader) -> dict:
    br.topics.sort()
    pp.pprint(br.topics)

    tof0_raw_data, tof1_raw_data = get_tof_raw_data(br=br)
    tof0_filtered_data, tof1_filtered_data = get_tof_filtered_data(br=br)
    wrench_data = get_wrench_data(br=br)
    joint_states_data = get_joint_states_data(br=br)
    tf_data = get_tf_data(br=br)
    tf_static_data = get_tf_data(br=br, static=True)
    fpc_transition_events_data = get_controller_events(br=br, controller_name="forward_position_controller")
    sjtc_transition_events_data = get_controller_events(br=br, controller_name="scaled_joint_trajectory_controller")

    df_dict = {
        "tof0_raw": create_df_from_data_dict(data=tof0_raw_data),
        "tof1_raw": create_df_from_data_dict(data=tof1_raw_data),
        "tof0_filtered": create_df_from_data_dict(data=tof0_filtered_data),
        "tof1_filtered": create_df_from_data_dict(data=tof1_filtered_data),
        "wrench": create_df_from_data_dict(data=wrench_data),
        "joint_states": create_df_from_data_dict(data=joint_states_data),
        "tf": create_df_from_data_dict(data=tf_data),
        "tf_static": create_df_from_data_dict(data=tf_static_data),
        "fpc_transition_events": create_df_from_data_dict(data=fpc_transition_events_data),
        "sjtc_transition_events": create_df_from_data_dict(data=sjtc_transition_events_data),
    }

    return df_dict


def filter_transition_events_for_fpc_deactivate(df: pd.DataFrame):
    # idxs_event_fpc_deactivate = df.index[df['fpc_transition_start_state'].fillna(-1).astype(int)==TransitionStates.TRANSITION_STATE_DEACTIVATING.value].to_list()

    # idxs_event_sjtc_deactivate = df.index[df['sjtc_transition_start_state'].fillna(-1).astype(int)==TransitionStates.TRANSITION_STATE_DEACTIVATING.value].to_list()
    # print(idxs_event_fpc_deactivate)
    # print(idxs_event_sjtc_deactivate)

    df_transition_events = df.loc[
        (
            df["controller_transition_start_state"].fillna(-1).astype(int)
            == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
        )
        # | (
        #     df["sjtc_transition_start_state"].fillna(-1).astype(int)
        #     == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
        # ),
        # [
        # "controller_transition_events_ts",
        # "controller_transition_start_state",
        # "controller_transition_goal_state",
        # "sjtc_transition_events_ts",
        # "sjtc_transition_start_state",
        # "sjtc_transition_goal_state",
        # ],
    ].reset_index()

    return df_transition_events


def get_bin_mask(df: pd.DataFrame, start_time: float, end_time: float, time_str: str):
    return (df[time_str] >= start_time) & (df[time_str] <= end_time)


def slice_df_by_transition_event(df: pd.DataFrame, df_name: str, transition_event_df: pd.DataFrame):
    # logger.info()
    i = 0
    start_index = 0
    end_index = start_index + 2
    last_iteration = False
    while not last_iteration:
        print(i)
        try:
            print(start_index, end_index)
            start_time = transition_event_df.at[start_index, "controller_transition_events_ts"]
            end_time = transition_event_df.at[end_index, "controller_transition_events_ts"]
        except Exception as e:
            start_time = transition_event_df.at[start_index, "controller_transition_events_ts"]
            end_time = np.inf
            last_iteration = True

        df_trial = df.loc[get_bin_mask(df=df, start_time=start_time, end_time=end_time, time_str=f"{df_name}_ts")]

        i += 1

        # hard code bad controller switch points when failures happened
        if i == 13:
            start_index = end_index
            end_index = start_index + 1
        else:
            start_index = end_index
            end_index = start_index + 2

        yield df_trial

    return


def warehouse_trials_dfs(trials_dfs: list[pd.DataFrame], topic_name: str, db_name: str):
    export_name = Path(Path(db_name).stem).stem
    trial_path = os.path.join(warehouse_path, export_name)
    if not os.path.exists(trial_path):
        os.mkdir(trial_path)

    for i, trial in enumerate(trials_dfs):
        trial.to_hdf(
            path_or_buf=os.path.join(trial_path, export_name + f"__{topic_name}__{str(i).zfill(3)}.h5"),
            key="data",
            format="fixed",
        )

    return


def warehouse_df(df: pd.DataFrame, topic_name: str, db_name: str):
    export_name = Path(Path(db_name).stem).stem
    trial_path = os.path.join(warehouse_path, export_name)
    if not os.path.exists(trial_path):
        os.mkdir(trial_path)
    df.to_hdf(path_or_buf=os.path.join(trial_path, export_name + f"__{topic_name}.h5"), key="data", format="fixed")
    return


def main():
    import sqlite3
    import traceback

    dbs = get_dbs(city="prosser", farm="roza", date="20250221")
    pp.pprint(dbs)
    # sys.exit()
    for db_name in dbs:
        if not db_name.endswith("bds__prosser_roza_t1.1.2__20250221_09-57-31_0.db3.zstd"):
            continue
        try:
            br = get_bag_reader(db=db_name)
        except sqlite3.DatabaseError as e:
            logger.error(f"Database read error: {traceback.format_exc()}")
            continue

        df_dict = get_dfs_from_bag_reader(br=br)
        br.cleanup()

        df_transition_events = filter_transition_events_for_fpc_deactivate(df=df_dict["fpc_transition_events"])

        for topic_df_name, topic_df in df_dict.items():
            if ("transition" in topic_df_name) or (topic_df_name == "tf_static"):
                warehouse_df(df=topic_df, topic_name=topic_df_name, db_name=db_name)
                continue
            trials_dfs = list(
                slice_df_by_transition_event(
                    df=topic_df, df_name=topic_df_name, transition_event_df=df_transition_events
                )
            )

            warehouse_trials_dfs(trials_dfs=trials_dfs, topic_name=topic_df_name, db_name=db_name)

    return


if __name__ == "__main__":
    main()
