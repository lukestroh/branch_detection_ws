#!/usr/bin/env python3
import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
import os
import scipy.signal as ssi

from branch_detection_system_analysis.plot import debug_plots as dplot

def circlular_distance(a1, a2):
    direct_dist = abs(a1 - a2)
    wraparound_dist = 2 * np.pi - direct_dist
    return min(direct_dist, wraparound_dist)


def filter_minima_by_angle_proximity(
    node, joint_states, distances, valley_idxs, angle_thresh=0.1, far_plane_filter=0.25
):
    # joint_states = np.unwrap(np.asarray(joint_states))  # unwrap for proximity comparisons
    distances = np.asarray(distances)

    sorted_idxs = valley_idxs[np.argsort(joint_states[valley_idxs])]
    filtered_idxs = []

    if distances[sorted_idxs[0]] > far_plane_filter:
        group = [sorted_idxs[1]]
        _idxs = sorted_idxs[2:]
    else:
        group = [sorted_idxs[0]]
        _idxs = sorted_idxs[1:]
    for idx in _idxs:
        # node.info(joint_states[idx])
        # Also filter by height, since we tacked on the end points
        if distances[idx] > far_plane_filter:
            continue
        prev_idx = group[-1]
        # if np.abs(joint_states[idx] - joint_states[prev_idx]) < angle_thresh:
        if circlular_distance(joint_states[idx], joint_states[prev_idx]) < angle_thresh:
            group.append(idx)
        else:
            group_arr = np.array(group)
            best_idx = group_arr[np.argmin(distances[group_arr])]
            filtered_idxs.append(best_idx)
            group = [idx]

    if group:
        group_arr = np.array(group)
        best_idx = group_arr[np.argmin(distances[group_arr])]
        filtered_idxs.append(best_idx)

    num_minima = len(filtered_idxs)
    if num_minima not in [2, 3]:
        node.error(
            f"Found too many minima: {num_minima}. Either multiple targets spotted, or consider adjusting angle threshold."
        )

    return np.asarray(filtered_idxs)


def detect_sectioned_window_indices(node, tof_data, joint_angles, filtered_valley_idxs, angle_thresh):

    def get_idx_midpoint_from_joint_angles(joint_angles, idx0, idx1):
        # Find midpoint between the two minima
        midpoint_angle = (joint_angles[idx0] + joint_angles[idx1]) / 2
        matches = np.where(np.isclose(joint_angles, midpoint_angle, atol=0.01))[0]
        if len(matches) == 0:
            # fallback if no exact match — just use the average of indices
            midpoint_idx = (idx0 + idx1) // 2
        else:
            midpoint_idx = matches[0]

        return midpoint_idx

    num_minima = len(filtered_valley_idxs)
    node.info(f"NUMBER MINIMA: {num_minima}")
    if num_minima == 2:
        # Standard case - split at midpoint between the two minima
        idx0, idx1 = sorted(filtered_valley_idxs)

        midpoint_idx = get_idx_midpoint_from_joint_angles(joint_angles=joint_angles, idx0=idx0, idx1=idx1)
        # node.info(f"MIDPOINT: {joint_angles[midpoint_idx]}")

        # Create two sections
        section0_idxs = np.arange(0, midpoint_idx + 1)
        section1_idxs = np.arange(midpoint_idx, len(joint_angles))
        return section0_idxs, section1_idxs, (idx0, idx1)

    elif num_minima == 3:
        # Sort minima by their index position (not angle)
        sorted_by_index = filtered_valley_idxs[np.argsort(filtered_valley_idxs)]

        # Get the angles at these minima
        angles_at_minima = joint_angles[sorted_by_index]
        

        # Calculate distances between consecutive minima in angle space
        # But also consider wraparound distances
        d_01 = circlular_distance(angles_at_minima[0], angles_at_minima[1])
        d_12 = circlular_distance(angles_at_minima[1], angles_at_minima[2])
        d_20 = circlular_distance(angles_at_minima[2], angles_at_minima[0])

        dists = [d_01, d_12, d_20]
        min_diff_idx = np.argmin(dists)

        if dists[min_diff_idx] < angle_thresh:
            # Get the indices of the three minima (sorted by index, not angle)
            m0, m1, m2 = sorted_by_index

            minima_midpoint0 = get_idx_midpoint_from_joint_angles(joint_angles=joint_angles, idx0=m0, idx1=m1)
            minima_midpoint1 = get_idx_midpoint_from_joint_angles(joint_angles=joint_angles, idx0=m1, idx1=m2)

            # node.info(f"MIDPOINT: {joint_angles[minima_midpoint0]}")
            # node.info(f"MIDPOINT: {joint_angles[minima_midpoint1]}")

            section0_idxs = np.concatenate(
                [np.arange(0, minima_midpoint0 + 1), np.arange(minima_midpoint1, len(joint_angles))]
            )
            section1_idxs = np.arange(minima_midpoint0, minima_midpoint1 + 1)

            # Ensure indices are unique and sorted
            section0_idxs = np.unique(section0_idxs)
            section1_idxs = np.unique(section1_idxs)
            return section0_idxs, section1_idxs, (m0, m1, m2)

        else:
            # filter out false positives from extra minima that don't wrap around the circle
            filtered_from_false_positives = np.argsort(tof_data[sorted_by_index][:2])
            min_indices = sorted_by_index[filtered_from_false_positives]
            midpoint_idx = get_idx_midpoint_from_joint_angles(joint_angles=joint_angles, idx0=min_indices[0], idx1=min_indices[1])
            # node.info(f"MIDPOINT: {joint_angles[midpoint_idx]}")

            # Create two sections
            section0_idxs = np.arange(0, midpoint_idx + 1)
            section1_idxs = np.arange(midpoint_idx, len(joint_angles))
            return section0_idxs, section1_idxs, min_indices
        
    return None


def separate_tof_data_by_curve(node, all_data_dict: dict, save_fig: bool = False, save_fig_path: str = "", debug_plot: bool = True, show_fig: bool = True):
    """After concatenation, split the tof data into two parabolic shapes. If only one exists, failure?"""
    # Get minima. We are searching for two
    joint_angles = all_data_dict["joint_states_data"][:, 2]
    tof_data = all_data_dict['tof_data']

    valley_idxs, heights_dict = ssi.find_peaks(
        x=(-1 * np.asarray(all_data_dict["tof_data"])),
        height=(-1 * node._param_far_plane_filter),
        # prominence=0.5,
        distance=40,
    )

    # Step 2: Add endpoints manually
    # endpoint_minima = []
    # if tof_data[0] < tof_data[1]:
    #     endpoint_minima.append(0)
    # if tof_data[-1] < tof_data[-2]:
    #     endpoint_minima.append(len(joint_angles) - 1)
    endpoint_minima = [0, len(joint_angles) - 1]
    valley_idxs = np.concatenate([valley_idxs, endpoint_minima]).astype(np.int64)
    valley_idxs = np.unique(valley_idxs)

    # node.warn(valley_idxs)
    # node.warn(joint_angles[valley_idxs])

    angle_threshold = np.radians(30)

    # Get minima, filter by proximity
    filtered_valley_idxs = filter_minima_by_angle_proximity(
        node=node,
        joint_states=joint_angles,
        distances=all_data_dict["tof_data"],
        valley_idxs=valley_idxs,
        angle_thresh=angle_threshold,
    )
    # node.warn(filtered_valley_idxs)
    # node.warn(joint_angles[filtered_valley_idxs])

    sectioned_idxs = detect_sectioned_window_indices(
        node=node, tof_data=tof_data, joint_angles=joint_angles, filtered_valley_idxs=filtered_valley_idxs, angle_thresh=angle_threshold
    )
    if sectioned_idxs is not None:
        section0_idxs, section1_idxs, split_idxs = sectioned_idxs
    else:
        return None
    
    # TODO: Do debug plot here
    if debug_plot:
        fig = dplot.plot_tof_vs_joint_state(data=all_data_dict, name="all_data")

        # Add minima to plot
        for i, idx in enumerate(valley_idxs):
            if i == 0:
                showlegend = True
            else:
                showlegend = False
            fig.add_trace(
                go.Scatter(
                    x=[all_data_dict["joint_states_data"][idx][2]],
                    y=[all_data_dict["tof_data"][idx]],
                    mode="markers",
                    name="minimum",
                    marker=dict(size=20, color="LightSkyBlue"),
                    showlegend=showlegend,
                    legendgroup="minima",
                    legendgrouptitle=dict(text="minima"),
                )
            )
        for i, idx in enumerate(filtered_valley_idxs):
            if i == 0:
                showlegend = True
            else:
                showlegend = False
            fig.add_trace(
                go.Scatter(
                    x=[all_data_dict["joint_states_data"][idx][2]],
                    y=[all_data_dict["tof_data"][idx]],
                    mode="markers",
                    name="filtered_minimum",
                    marker=dict(size=20, color="orange"),
                    showlegend=showlegend,
                    legendgroup="filtered_minima",
                    legendgrouptitle=dict(text="filtered_minima"),
                )
            )
        if save_fig:
            pio.write_html(
                fig=fig,
                file=os.path.join(save_fig_path, "tof_vs_joint_state_all_data.html"),
                auto_open=show_fig,
            )
        else:
            if show_fig:
                fig.show()

    separated_data_dict = {"s0": {}, "s1": {}}
    separated_data_dict["s0"]["raw_tof_ts"] = all_data_dict["raw_tof_ts"][section0_idxs]
    separated_data_dict["s0"]["raw_tof_data"] = all_data_dict["raw_tof_data"][section0_idxs]
    separated_data_dict["s0"]["tof_ts"] = all_data_dict["tof_ts"][section0_idxs]
    separated_data_dict["s0"]["tof_data"] = all_data_dict["tof_data"][section0_idxs]
    separated_data_dict["s1"]["raw_tof_ts"] = all_data_dict["raw_tof_ts"][section1_idxs]
    separated_data_dict["s1"]["raw_tof_data"] = all_data_dict["raw_tof_data"][section1_idxs]
    separated_data_dict["s1"]["tof_ts"] = all_data_dict["tof_ts"][section1_idxs]
    separated_data_dict["s1"]["tof_data"] = all_data_dict["tof_data"][section1_idxs]

    separated_data_dict["s0"]["joint_states_ts"] = all_data_dict["joint_states_ts"][section0_idxs]
    separated_data_dict["s0"]["joint_states_data"] = all_data_dict["joint_states_data"][section0_idxs]
    separated_data_dict["s1"]["joint_states_ts"] = all_data_dict["joint_states_ts"][section1_idxs]
    separated_data_dict["s1"]["joint_states_data"] = all_data_dict["joint_states_data"][section1_idxs]

    separated_data_dict["s0"]["indices"] = section0_idxs
    separated_data_dict["s1"]["indices"] = section1_idxs

    # TODO: another debug plot here
    if debug_plot:
        fig = dplot.plot_tof_vs_joint_state(data=separated_data_dict["s0"], name="s0")
        fig = dplot.plot_tof_vs_joint_state(data=separated_data_dict["s1"], name="s1", fig=fig)
        if save_fig:
            pio.write_html(
                fig=fig,
                file=os.path.join(save_fig_path, "tof_vs_joint_state_by_section.html"),
                auto_open=show_fig,
            )
        else:
            if show_fig:
                fig.show()

    return separated_data_dict


def amend_joint_angle_discontinuity(node, joint_angles: np.ndarray, indices: np.ndarray) -> np.ndarray:
    """
    :param joint_angles: A array of joint angles
    :type joint_angles: np.ndarray
    :param indices: A array of indices corresponding to the joint angles
    :type indices: np.ndarray
    :returns: An array of continuous joint angles
    :rtype: ndarray
    """
    idx_diffs = np.diff(indices)
    gap_mask = idx_diffs > 1

    if not np.any(gap_mask):
        node.warn("No gap detected.")
        return joint_angles
    # Find largest gap, split the joint angle data at the gap.
    gap_idx = np.argmax(idx_diffs)
    new_indices = np.arange(len(joint_angles))
    first_part_indices = new_indices[: gap_idx + 1]
    second_part_indices = new_indices[gap_idx + 1 :]

    first_part_angles = joint_angles[first_part_indices]
    second_part_angles = joint_angles[second_part_indices]

    # Determine which part is further from 0 (this part should be shifted)
    first_part_distance_from_zero = np.mean(np.abs(first_part_angles))
    second_part_distance_from_zero = np.mean(np.abs(second_part_angles))

    if first_part_distance_from_zero > second_part_distance_from_zero:
        node.warn("shifting left side")
        # Shift first part
        shift_direction = -1 if np.mean(first_part_angles) > 0 else 1
        shifted_angles = first_part_angles + shift_direction * 2 * np.pi

        # Check if shift keeps us in [-2π, 2π] range
        if np.any(shifted_angles > 2 * np.pi) or np.any(shifted_angles < -2 * np.pi):
            # Try opposite direction
            shift_direction *= -1
            shifted_angles = first_part_angles + shift_direction * 2 * np.pi

        # Combine: shifted first part + original second part
        return np.concatenate([shifted_angles, second_part_angles])

    else:
        node.warn("shifting right side")
        # Shift second part
        shift_direction = -1 if np.mean(second_part_angles) > 0 else 1
        shifted_angles = second_part_angles + shift_direction * 2 * np.pi

        # Check if shift keeps us in [-2π, 2π] range
        if np.any(shifted_angles > 2 * np.pi) or np.any(shifted_angles < -2 * np.pi):
            # Try opposite direction
            shift_direction *= -1
            shifted_angles = second_part_angles + shift_direction * 2 * np.pi

        # Combine: original first part + shifted second part
        
        return np.concatenate([first_part_angles, shifted_angles])
