#!/usr/bin/env python3

import plotly.graph_objects as go
import numpy as np


def plot_vector(
    fig: go.Figure,
    position: np.ndarray,
    orientation: np.ndarray,
    scale: float,
    color: str,
    anchor: str = "tail",
    name: str = "",
):

    fig.add_trace(
        go.Cone(
            x=[position[0]],
            y=[position[1]],
            z=[position[2]],
            u=[orientation[0]],
            v=[orientation[1]],
            w=[orientation[2]],
            name=name,
            sizemode="scaled",
            sizeref=0.1 * scale,
            showscale=False,
            showlegend=False,
            anchor=anchor,
            colorscale=[[0, color], [1, color]],
        )
    )

    return fig


def plot_3d_coordinate_frame(fig: go.Figure, position: np.ndarray, orientation: np.ndarray, name: str = "origin"):
    # if name in self.coordinate_frames:
    #     raise ValueError(f"Coordinate frame with name '{name}' already exists.")
    # else:
    #     self.coordinate_frames = [name]
    zoom_scale = 0.05
    cone_scale = 0.25
    axes = {
        "x-axis": {
            "x": np.array([position[0], position[0] + zoom_scale]),
            "y": np.array([position[1], position[1] + 0]),
            "z": np.array([position[2], position[2] + 0]),
            "u": np.array([zoom_scale * cone_scale]),
            "v": np.array([0]),
            "w": np.array([0]),
            "color": "red",
        },
        "y-axis": {
            "x": np.array([position[0], position[0] + 0]),
            "y": np.array([position[1], position[1] + zoom_scale]),
            "z": np.array([position[2], position[2] + 0]),
            "u": np.array([0]),
            "v": np.array([zoom_scale * cone_scale]),
            "w": np.array([0]),
            "color": "green",
        },
        "z-axis": {
            "x": np.array([position[0], position[0] + 0]),
            "y": np.array([position[1], position[1] + 0]),
            "z": np.array([position[2], position[2] + zoom_scale]),
            "u": np.array([0]),
            "v": np.array([0]),
            "w": np.array([zoom_scale * cone_scale]),
            "color": "blue",
        },
    }

    for axis, val in axes.items():
        fig.add_trace(
            go.Scatter3d(
                x=axes[axis]["x"],
                y=axes[axis]["y"],
                z=axes[axis]["z"],
                name=f"{name}_{axis}",
                mode="lines",
                line=dict(width=3, color=axes[axis]["color"]),
                showlegend=False,
                legendgroup=f"{name}_{axis}",
                legendgrouptitle=dict(text=name),
            )
        )

        fig.add_trace(
            go.Cone(
                x=[axes[axis]["x"][1]],
                y=[axes[axis]["y"][1]],
                z=[axes[axis]["z"][1]],
                u=axes[axis]["u"],
                v=axes[axis]["v"],
                w=axes[axis]["w"],
                name=axis,
                showscale=False,
                colorscale=[[0, axes[axis]["color"]], [1, axes[axis]["color"]]],
                anchor="tail",
                sizemode="scaled",
                legendgroup=f"{name}_{axis}",
                legendgrouptitle=dict(text=name),
                showlegend=False,
                # hoverinfo="skip",
                # hovertemplate=None
            )
        )
    return fig


def main():

    fig = go.Figure()

    fig = plot_3d_coordinate_frame(fig=fig, position=np.zeros(3), orientation=None)

    fig = plot_vector(fig=fig, position=[1, 1, 1], orientation=[1, 1, 1], scale=0.5)

    fig.show()

    return


if __name__ == "__main__":
    main()
