#!/usr/bin/env python3
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from branch_detection_system_analysis.plot import plotting_backend as pb


def get_tof_vec_base_frame(
    node,
    data_dict: dict[pd.DataFrame],
    time_and_center_res_dict: dict,
    section_name: str,
):
    # If the eef is moving, we need a common frame, which should be world or <robot-part>__base
    # Get tof poses at calculated signal minimum times
    timestamp = time_and_center_res_dict[section_name]['time']
    dist = time_and_center_res_dict[section_name]['min_dist']
    sensor_id = time_and_center_res_dict[section_name]['sensor_id']

    tf_df = pb.get_tf_df_at_closest_timestamp(
        tf_df=data_dict["tf"], tf_static_df=data_dict["tf_static"], timestamp=timestamp
    )

    tf_tof_to_base = pb.get_tf_matrix_from_df(
        target_frame=f"{node._param_robot_base_part}__base", source_frame=f"{node._param_robot_eef_part}__tof{sensor_id}", tf_df=tf_df
    )
    vec = tf_tof_to_base @ [0,0,dist,1]

    return vec


def get_branch_vec_from_tof(node, data_dict: dict[pd.DataFrame], time_and_center_res_dict:dict, return_frames: bool = False):
    # Project tof readings in base frame
    A_vec_base_frame = get_tof_vec_base_frame(
        node=node,
        data_dict=data_dict,
        time_and_center_res_dict=time_and_center_res_dict,
        section_name='s0'
    )
    B_vec_base_frame = get_tof_vec_base_frame(
        node=node,
        data_dict=data_dict,
        time_and_center_res_dict=time_and_center_res_dict,
        section_name='s1'
    )

    # Get the centerpoint of these two points.
    branch_center_point = np.mean([A_vec_base_frame, B_vec_base_frame], axis=0)  # C

    branch_vec = A_vec_base_frame - B_vec_base_frame
    branch_vec_normalized = branch_vec / np.linalg.norm(branch_vec)  # N

    if return_frames:
        return branch_center_point, branch_vec_normalized, A_vec_base_frame, B_vec_base_frame
    else:
        return branch_center_point, branch_vec_normalized, None, None


def get_desired_position_from_branch_vec(node, branch_center_point: np.ndarray, branch_vec: np.ndarray, time: float, data_dict: dict,):
    """
    Get closest point on a circle from point, given circle center,
    point, plane normal
    https://www.geometrictools.com/Documentation/DistanceToCircle3.pdf

    :param branch_vec: Unit vector of the branch in robot base domain.
    :type branch_vec: np.ndarray
    """
    tf_df = pb.get_tf_df_at_closest_timestamp(
        tf_df=data_dict["tf"], tf_static_df=data_dict["tf_static"], timestamp=time
    )

    tf_cut_point_to_base = pb.get_tf_matrix_from_df(
        target_frame=f"{node._param_robot_base_part}__base", source_frame=f"{node._param_robot_eef_part}__tool0", tf_df=tf_df
    )

    curr_pose = tf_cut_point_to_base[0:3, 3]  # P

    delta = curr_pose - branch_center_point[:3]
    _Q_C = delta - np.dot(branch_vec[:3], delta) * branch_vec[:3]

    desired_radius_from_branch = 0.10  # m

    desired_eef_xyz = branch_center_point[:3] + _Q_C / np.linalg.norm(_Q_C) * desired_radius_from_branch
    return desired_eef_xyz


def get_desired_orientation_from_branch_vec(branch_center_point, branch_vec, desired_eef_xyz, return_vec: bool = False):
    """Creates a set of basis vectors defining the desired coordinate system and returns a quaternion from the robot base frame"""
    desired_orientation_vec_to_branch = branch_center_point[:3] - desired_eef_xyz
    desired_orientation_vec_to_branch_norm = desired_orientation_vec_to_branch / np.linalg.norm(
        desired_orientation_vec_to_branch
    )
    desired_y_axis = np.cross(desired_orientation_vec_to_branch_norm, branch_vec[:3])
    # Form the rotation matrix from our basis vectors
    rot_mat = np.column_stack((branch_vec[:3], desired_y_axis, desired_orientation_vec_to_branch_norm))
    desired_orientation_rot = Rotation.from_matrix(rot_mat)
    desired_orientation_quat = desired_orientation_rot.as_quat()

    return desired_orientation_quat, desired_orientation_vec_to_branch_norm
