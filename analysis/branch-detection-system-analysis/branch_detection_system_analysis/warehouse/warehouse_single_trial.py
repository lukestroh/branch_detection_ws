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
from final_approach_controller_msgs.msg import GeneratedPoses, TimestampTofMin
from geometry_msgs.msg import WrenchStamped
from lifecycle_msgs.msg import TransitionEvent, State
from ism330dhcx_msgs.msg import Ism330dhcxStamped
from tof_msgs.msg import TofStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from vl53l4cd_msgs.msg import Vl53l4cdStamped

import sys
import pprint as pp


import rclpy.logging

logger = rclpy.logging.get_logger("warehouser")


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
bags_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection")
warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))


def get_dbs(location: str, farm: str, date: str = "") -> list[str]:
    if date != "":
        files = glob.glob(bags_path + f"/**/*{location}_{farm}*__{date}*.db3.zstd")
    else:
        files = glob.glob(bags_path + f"/**/*{location}_{farm}*.db3.zstd")
    return files


def get_dbs_by_loc(location: str) -> list[str]:
    files = glob.glob(bags_path + f"/**/*{location}*.zstd")
    return files


def get_bag_reader(db: str) -> BagReader:
    return BagReader(bag_file=db)


def get_tof_raw_data(br: BagReader) -> tuple[dict]:
    try:
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
    except (KeyError, ValueError):
        tof0_raw_ts = tof0_raw_data = tof1_raw_ts = tof1_raw_data = []

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
    try:
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
    except (KeyError, ValueError):
        tof0_filtered_ts = tof0_filtered_data = tof1_filtered_ts = tof1_filtered_data = []

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


def get_imu_stamped_data(br: BagReader) -> tuple[dict]:
    try:
        data_imu_stamped = list(br.query(topic_name="/ism330dhcx_stamped"))
        imu_stamped: list[Ism330dhcxStamped] = [d[1] for d in data_imu_stamped]
        imu_ts, imu_data_ax, imu_data_ay, imu_data_az = zip(
            *map(
                lambda imu: [
                    imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9,
                    imu.linear_acceleration.x,
                    imu.linear_acceleration.y,
                    imu.linear_acceleration.z,
                ],
                imu_stamped,
            )
        )
    except (ValueError, KeyError):
        imu_ts = imu_data_ax = imu_data_ay = imu_data_az = []
    return {
        "imu_ts": imu_ts,
        "imu_ax": imu_data_ax,
        "imu_ay": imu_data_ay,
        "imu_az": imu_data_az,
    }


def get_generated_start_poses_data(br: BagReader) -> dict:
    try:
        _generated_start_poses = list(br.query(topic_name=f"/generated_start_poses"))
        generated_start_poses: list[GeneratedPoses] = [d[1] for d in _generated_start_poses]
        start_poses = zip(*map(lambda g: g.poses, generated_start_poses))
    except (KeyError, ValueError):
        start_poses = []
    return {"start_poses": start_poses}


def get_controller_success(br: BagReader, controller_name: str, topic: str) -> dict:
    try:
        _controller_success = list(br.query(topic_name=f"/{controller_name}_controller/{topic}_success"))
        controller_success: list[bool] = [d[1].data for d in _controller_success]
    except (KeyError, ValueError):
        controller_success = []
    return {"controller_success": controller_success}


"""
'/fbrw_controller/ts_tof_min',

"""


def get_ts_tof_min(br: BagReader) -> dict:
    try:
        _ts_tof_min = list(br.query(topic_name="/fbrw_controller/ts_tof_min"))
        ts_ts_tof_min = [d[0] for d in _ts_tof_min]
        ts_tof_min: list[TimestampTofMin] = [d[1] for d in _ts_tof_min]
        sensor_id, ts_min, data_min = zip(*map(lambda t: [t.sensor_id, t.timestamp, t.data], ts_tof_min))
    except (KeyError, ValueError):
        ts_ts_tof_min = sensor_id = ts_min = data_min = []
    return {"msg_stamp": ts_ts_tof_min, "sensor_id": sensor_id, "ts_min": ts_min, "data_min": data_min}


def get_wrench_data(br: BagReader) -> dict:
    # FT-wrench data
    try:
        data_ft_wrench = list(br.query(topic_name="/force_torque_sensor_broadcaster/wrench"))
        wrench_data: list[WrenchStamped] = [d[1] for d in data_ft_wrench]
        (
            wrench_data_ts,
            wrench_data_fx,
            wrench_data_fy,
            wrench_data_fz,
            wrench_data_tx,
            wrench_data_ty,
            wrench_data_tz,
        ) = zip(
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
    except (KeyError, ValueError):
        wrench_data_ts = wrench_data_fx = wrench_data_fy = wrench_data_fz = wrench_data_tx = wrench_data_ty = (
            wrench_data_tz
        ) = []
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
    try:
        joint_states = list(br.query(topic_name="/joint_states"))
        joint_states_data: list[JointState] = [d[1] for d in joint_states]

        joint_states_ts, joint_states_pos = zip(
            *map(lambda ja: [ja.header.stamp.sec + ja.header.stamp.nanosec * 1e-9, ja.position], joint_states_data)
        )
    except (KeyError, ValueError):
        joint_states_ts = joint_states_pos = []

    return {"joint_states_ts": joint_states_ts, "joint_states_pos": np.array(joint_states_pos)}


def get_tf_data(br: BagReader, static=False) -> dict:
    """
    Gets transform data. Set `static=True` to get static transform data

    :param br: BagReader object for database extraction.
    :type br: BagReader
    :param static: Set true to get static transforms from topic '/tf_static'. Defaults to false.
    :type static: bool
    :returns: A dictionary of all transform data
    :rtype: dict
    """
    if static:
        topic_name = "tf_static"
    else:
        topic_name = "tf"

    try:
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
    except (KeyError, ValueError):
        tf_ts = tf_frame_id = tf_child_frame_id = tf_t_x = tf_t_y = tf_t_z = tf_r_x = tf_r_y = tf_r_z = tf_r_w = []

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
    try:
        controller_events = list(br.query(topic_name=f"/{controller_name}/transition_event"))
        controller_transition_events_ts, controller_events_data = zip(
            *map(lambda ctrlr: [ctrlr[0] * 1e-9, ctrlr[1]], controller_events)
        )

        controller_transition_start_state, controller_transition_goal_state = zip(
            *map(lambda ctrlr_t: [ctrlr_t.start_state.id, ctrlr_t.goal_state.id], controller_events_data)
        )
    except (KeyError, ValueError):
        controller_transition_events_ts = controller_transition_start_state = controller_transition_goal_state = []

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
    imu_data = get_imu_stamped_data(br=br)
    wrench_data = get_wrench_data(br=br)
    joint_states_data = get_joint_states_data(br=br)
    tf_data = get_tf_data(br=br)
    tf_static_data = get_tf_data(br=br, static=True)
    start_poses_data = get_generated_start_poses_data(br=br)
    fbwr_controller_localization_success_data = get_controller_success(br=br, controller_name="fbrw", topic="alignment")
    fbwr_controller_alignment_success_data = get_controller_success(br=br, controller_name="fbrw", topic="localization")
    ts_tof_min_data = get_ts_tof_min(br=br)
    fpc_transition_events_data = get_controller_events(br=br, controller_name="forward_position_controller")
    sjtc_transition_events_data = get_controller_events(br=br, controller_name="scaled_joint_trajectory_controller")

    data_dict = {
        "tof0_raw": tof0_raw_data,
        "tof1_raw": tof1_raw_data,
        "tof0_filtered": tof0_filtered_data,
        "tof1_filtered": tof1_filtered_data,
        "imu": imu_data,
        "wrench": wrench_data,
        "joint_states": joint_states_data,
        "tf": tf_data,
        "tf_static": tf_static_data,
        "start_poses": start_poses_data,
        "fbrw_controller_localization_success": fbwr_controller_localization_success_data,
        "fbrw_controller_alignment_success": fbwr_controller_alignment_success_data,
        "ts_tof_min": ts_tof_min_data,
        "fpc_transition_events": fpc_transition_events_data,
        "sjtc_transition_events": sjtc_transition_events_data,
    }

    df_dict = {k: create_df_from_data_dict(data=v) for k, v in data_dict.items()}

    return df_dict


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


def warehouse_trial_df(trial_df: pd.DataFrame, topic_name: str, db_name: str):
    export_name = Path(Path(db_name).stem).stem
    trial_path = os.path.join(warehouse_path, export_name)
    if not os.path.exists(trial_path):
        os.mkdir(trial_path)

    trial_df.to_hdf(
        path_or_buf=os.path.join(trial_path, export_name + f"__{topic_name}__0.h5"),
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


def is_already_warehoused(compressed_db_name: str) -> bool:
    warehouse_parent = Path(Path(compressed_db_name).stem).stem
    p = os.path.join(warehouse_path, warehouse_parent)
    if os.path.exists(p):
        logger.warn(f"Warehouse {p} already exists, skipping file.")
        return True
    else:
        return False


def get_metadata_and_debug_files(compressed_db_name: str) -> list[str]:
    warehouse_parent = Path(Path(compressed_db_name).stem).stem
    p = os.path.join(warehouse_path, warehouse_parent)
    glob_ignored_files = (".zstd", ".db3")
    all_files = glob.glob(p + "/*")

    files = [file for file in all_files if not file.endswith(glob_ignored_files)]

    return files


def main():
    from branch_detection_system_analysis.plot import plotting_backend as pb
    import sqlite3
    import traceback
    import sys

    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse")

    dbs = get_dbs(location="prosser", farm="allen", date="20250220")
    # dbs = get_dbs_by_loc(location="arm_farm")
    # sys.exit()

    # trial_file_name = "bds__arm_farm__20250519_15-27-56"

    # dbs = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name=trial_file_name)
    # print(dbs)
    for db_name in dbs:
        # if trial_file_name not in db_name:
        #     continue
        if is_already_warehoused(compressed_db_name=db_name):
            continue
        else:
            try:
                br = get_bag_reader(db=db_name)
            except sqlite3.DatabaseError as e:
                logger.error(f"Database read error: {traceback.format_exc()}")
                continue
            # try:
            df_dict = get_dfs_from_bag_reader(br=br)
            # except (ValueError, KeyError) as e:
            #     logger.error(f"{traceback.format_exc()}")
            #     continue
            br.cleanup()

            for topic_df_name, topic_df in df_dict.items():
                warehouse_trial_df(trial_df=topic_df, topic_name=topic_df_name, db_name=db_name)

    return


if __name__ == "__main__":
    main()
