#!/usr/bin/env python3
import branch_detection_system_analysis.plot.plotting_backend as pb


import plotly.graph_objects as go
import numpy as np


def plot_tof_vs_joint_state(df_dict: dict, tof_name: str, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    tof_df = df_dict[f"{tof_name}_filtered"]

    timestamps = tof_df[f"{tof_name}_filtered_ts"].to_numpy()

    joint_states_ts_filtered_df = pb.get_df_rows_at_closest_timestamp(
        df_dict=df_dict, topic_name="joint_states", timestamps=timestamps
    )

    wrist_3_pos = np.vstack(joint_states_ts_filtered_df["joint_states_pos"])[:, 2]

    fig.add_trace(
        go.Scatter(
            x=wrist_3_pos,
            y=tof_df[f"{tof_name}_filtered_data"],
            mode="markers",
        )
    )

    fig.update_layout(
        title=dict(text=f"{tof_name} MAF readings vs. ur5e__wrist_3 position"),
        xaxis=dict(title="Wrist-3 position"),
        yaxis=dict(title="Distance (m)"),
    )

    fig.show()

    return fig
