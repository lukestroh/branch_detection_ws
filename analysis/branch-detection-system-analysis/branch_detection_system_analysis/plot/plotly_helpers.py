#!/usr/bin/env python3

import plotly.graph_objects as go
import numpy as np
from numpy.typing import ArrayLike



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


def plot_3d_coordinate_frame(
    fig: go.Figure,
    position: np.ndarray,
    orientation: np.ndarray,
    axis_length: float = 0.05,
    cone_scale: float = 0.25,
    name: str = "",
    parent_frame: str = "",
):
    """
    Plots a
    """
    # zoom_scale = 0.05
    # cone_scale = 0.25
    colors = {"x": "red", "y": "green", "z": "blue"}
    unit_vectors = {"x": np.array([1, 0, 0]), "y": np.array([0, 1, 0]), "z": np.array([0, 0, 1])}
    """
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
    """

    for axis_name, unit_vec in unit_vectors.items():
        direction = orientation @ unit_vec
        pos_end = position + axis_length * direction

        fig.add_trace(
            go.Scatter3d(
                x=[position[0], pos_end[0]],
                y=[position[1], pos_end[1]],
                z=[position[2], pos_end[2]],
                mode="lines",
                line=dict(width=3, color=colors[axis_name]),
                name=f"{name}__{axis_name}-axis",
                legendgroup=f"{name}_{axis_name}",
                legendgrouptitle=dict(text=name),
            )
        )

        fig.add_trace(
            go.Cone(
                x=[pos_end[0]],
                y=[pos_end[1]],
                z=[pos_end[2]],
                u=[cone_scale * direction[0]],
                v=[cone_scale * direction[1]],
                w=[cone_scale * direction[2]],
                anchor="tail",
                sizemode="scaled",
                sizeref=0.1 * cone_scale,
                showscale=False,
                colorscale=[[0, colors[axis_name]], [1, colors[axis_name]]],
                name=f"{name}__{axis_name}-axis",
                legendgroup=f"{name}_{axis_name}",
                legendgrouptitle=dict(text=name),
                showlegend=False,
            )
        )

    return fig


def plot_circle(center: ArrayLike, radius: float, name: str = "", color: str = "black", fig: go.Figure = None):
    if fig is None:
        fig = go.Figure()

    theta = np.linspace(0, 2 * np.pi, 360)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)

    fig.add_trace(go.Scatter(x=x, y=y, mode="lines", line=dict(color=color), name=name))
    return fig


def plot_circle_3d(center: ArrayLike, radius: float, name: str = "", color: str = "black", fig: go.Figure = None):
    """TODO: be able to position circle in any plane"""
    if fig is None:
        fig = go.Figure()

    theta = np.linspace(0, 2 * np.pi, 360)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)

    fig.add_trace(go.Scatter3d(x=x, y=y, z=np.full(len(x), 0.0), mode="lines", line=dict(color=color), name=name))
    return fig


def plot_cylinder(
    center: ArrayLike,
    orientation: ArrayLike,
    radius,
    height,
    nt: int = 100,
    nh: int = 50,
    name: str = "",
    color: str = "#ffffff",
    fig: go.Figure = None,
) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    # first, create a cylinder that lies along the z-axis, then rotate and translate the points
    _theta = np.linspace(0, 2 * np.pi, nt)
    _z = np.linspace(-height / 2, height / 2, nh)

    theta, z_local = np.meshgrid(_theta, _z)

    x_local = radius * np.cos(theta)
    y_local = radius * np.sin(theta)

    z_axis = np.array([0, 0, 1])

    # ensure orientation is unit vector
    orientation = np.asarray(orientation) / np.linalg.norm(orientation)

    if np.allclose(orientation, z_axis):
        # Already aligned
        rotation_matrix = np.eye(3)
    elif np.allclose(orientation, -z_axis):
        # Opposite direction, rotate 180° around x-axis
        rotation_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    else:
        v = np.cross(z_axis, orientation)
        s = np.linalg.norm(v)
        c = np.dot(z_axis, orientation)

        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

        rotation_matrix = np.eye(3) + vx + np.dot(vx, vx) * ((1 - c) / (s**2))

    # Apply rotation and translation
    points_local = np.stack([x_local.flatten(), y_local.flatten(), z_local.flatten()])
    points_global = rotation_matrix @ points_local

    # Reshape back to grid form
    X_global = points_global[0].reshape(x_local.shape) + center[0]
    Y_global = points_global[1].reshape(y_local.shape) + center[1]
    Z_global = points_global[2].reshape(z_local.shape) + center[2]

    fig.add_trace(
        go.Surface(
            x=X_global,
            y=Y_global,
            z=Z_global,
            showscale=False,
            opacity=0.7,
            name=name,
            surfacecolor=np.ones_like(X_global),  # Constant color values
            colorscale=[[0, color], [1, color]],  # Single color
            showlegend=True,
        )
    )

    fig.update_layout(
        scene=dict(aspectmode="data"),
    )

    return fig


def plot_plane_from_point_and_normal_vec(
    point: np.ndarray, norm: np.ndarray, plane_size: int = 1, color: str = "#731c1c", fig: go.Figure = None
):
    if fig is None:
        fig = go.Figure()

    x_vals = np.linspace(-plane_size / 2, plane_size / 2, 10)
    y_vals = np.linspace(-plane_size / 2, plane_size / 2, 10)
    x, y = np.meshgrid(x_vals, y_vals)
    z = compute_z_on_plane(x=x, y=y, norm=norm, point_on_plane=point)

    # Create the surface
    fig.add_trace(go.Surface(x=x, y=y, z=z, opacity=0.6, colorscale=[[0, color], [1, color]], showscale=False, showlegend=True, name='branch_plane'))

    return fig


def compute_z_on_plane(x, y, norm, point_on_plane):
    a, b, c = norm
    x0, y0, z0 = point_on_plane

    if c == 0:
        raise ValueError("The plane is vertical in z (normal.z = 0), z is undefined for given x, y")

    # Plane equation: a(x - x0) + b(y - y0) + c(z - z0) = 0
    z = ((-a * (x - x0)) - (b * (y - y0))) / c + z0
    return z


def main():

    fig = go.Figure()

    fig = plot_3d_coordinate_frame(fig=fig, position=np.zeros(3), orientation=None)

    fig = plot_vector(fig=fig, position=[1, 1, 1], orientation=[1, 1, 1], scale=0.5)

    fig.show()

    return


if __name__ == "__main__":
    main()
