#!/usr/bin/env python3


def plot_dict_data(data: dict, filename: str, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = plotly.subplots.make_subplots(shared_xaxes=True, specs=[[{"secondary_y": True}]])

    fig.add_trace(
        go.Scatter(
            x=data["tof0_filtered"]["tof0_filtered_ts"],
            y=data["tof0_filtered"]["tof0_filtered_data"],
            mode="markers",
            name="tof0_filtered",
        ),
        secondary_y=False,
    )
    fig.add_trace(
        go.Scatter(
            x=data["tof1_filtered"]["tof1_filtered_ts"],
            y=data["tof1_filtered"]["tof1_filtered_data"],
            mode="markers",
            name="tof1_filtered",
        ),
        secondary_y=False,
    )

    fig.update_layout(title=dict(text=Path(filename).stem))
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Distance (m)", secondary_y=False)
    fig.update_yaxes(title_text="Force (N)", secondary_y=True)

    return fig


def plot_data(df: pd.DataFrame, filename: str, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = plotly.subplots.make_subplots(shared_xaxes=True, specs=[[{"secondary_y": True}]])

    fig.add_trace(
        go.Scatter(
            x=df["tof0_filtered_ts"],
            y=df["tof0_filtered_data"],
            mode="markers",
            name="tof0_filtered",
        ),
        secondary_y=False,
    )
    fig.add_trace(
        go.Scatter(x=df["tof1_filtered_ts"], y=df["tof1_filtered_data"], mode="markers", name="tof1_filtered"),
        secondary_y=False,
    )
    # fig.add_trace(go.Scatter(x=df["wrench_ts"], y=df["wrench_fx"], mode="lines", name="wrench_fx"), secondary_y=True)
    # fig.add_trace(go.Scatter(x=df["wrench_ts"], y=df["wrench_fy"], mode="lines", name="wrench_fy"), secondary_y=True)
    # fig.add_trace(go.Scatter(x=df["wrench_ts"], y=df["wrench_fz"], mode="lines", name="wrench_fz"), secondary_y=True)

    fig.update_layout(title=dict(text=Path(filename).stem))
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Distance (m)", secondary_y=False)
    fig.update_yaxes(title_text="Force (N)", secondary_y=True)

    return fig


def plot_transition_events(df: pd.DataFrame, fig: go.Figure) -> go.Figure:
    df_fpc_transitions = df.loc[
        df["fpc_transition_start_state"] == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
    ]

    df_sjtc_transitions = df.loc[
        df["sjtc_transition_start_state"] == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
    ]

    for i, timestamp in enumerate(df_fpc_transitions["fpc_transition_events_ts"]):
        fig.add_vline(x=timestamp, line_width=2, line_dash="dash", line_color="green")
        fig.add_annotation(
            x=timestamp, text=f"fpc_event_{i}", showarrow=True, xanchor="left", yanchor="middle", textangle=-90
        )
    for i, timestamp in enumerate(df_sjtc_transitions["sjtc_transition_events_ts"]):
        fig.add_vline(x=timestamp, line_width=2, line_dash="dash", line_color="red")
        fig.add_annotation(
            x=timestamp, text=f"sjtc_event_{i}", showarrow=True, xanchor="left", yanchor="middle", textangle=-90
        )
    return fig
