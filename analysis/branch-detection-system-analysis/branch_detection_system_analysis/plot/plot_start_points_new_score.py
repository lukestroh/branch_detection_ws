#!/usr/bin/env python3
import branch_detection_system_analysis.plot.plotly_helpers as ph
import branch_detection_system_analysis.plot.ray_tracing as rt

import itertools
import plotly.graph_objects as go
import numpy as np
from numpy.typing import ArrayLike
from scipy.spatial.transform import Rotation

from typing import Optional


def generate_cylindrical_pts(
    r_range: ArrayLike,
    theta_range: ArrayLike,
    z_range: ArrayLike,
    num_r_pts: int,
    num_theta_pts: int,
    num_z_pts: int,
    start_point: np.ndarray,
    start_orientation: np.ndarray,
):
    r = np.linspace(r_range[0], r_range[1], num_r_pts)
    theta = np.linspace(theta_range[0], theta_range[1], num_theta_pts)
    z = np.linspace(z_range[0], z_range[1], num_z_pts)
    x = np.outer(r, np.cos(theta)).flatten()
    y = np.outer(r, np.sin(theta)).flatten()
    xy = np.stack((x, y), axis=1)
    xy_repeated = np.tile(xy, reps=(len(z), 1))
    z_repeated = np.repeat(z, len(x))[:, np.newaxis]
    xyz = np.hstack((xy_repeated, z_repeated))
    z_axis = np.array([0, 0, 1])
    orientation = start_orientation / np.linalg.norm(start_orientation)
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

    points_global = xyz @ rotation_matrix + start_point

    return points_global


def compute_plane_from_pts_and_orientation(p0, p1, v0):
    w = p1 - p0
    n = np.cross(v0, w)
    return n / np.linalg.norm(n)


def sample_gaussian_cone(
    u: np.ndarray,  # points towards branch
    v: np.ndarray,  # other basis vectors
    w: np.ndarray,
    sensor_fov_deg: float,
    sigma_deg: float,
    num_samples: int,
    seed: int | None = None,
):
    # Convert to rad
    sensor_fov = np.radians(sensor_fov_deg)
    sigma = np.radians(sigma_deg)
    max_radius = np.tan(sensor_fov / 2)

    rgen = np.random.default_rng(seed=seed)

    sampled_directions = []
    while len(sampled_directions) < num_samples:
        # 2D Gaussian on tangent plane
        x, y = rgen.normal(loc=0, scale=sigma / 3, size=2)
        # Reject outside circular aperture
        if np.hypot(x, y) > max_radius:
            continue
        # Build direction: tilt u by (x, y) in v, w
        T = x * v + y * w
        d = u + T
        d /= np.linalg.norm(d)
        sampled_directions.append(d)
    return np.asarray(sampled_directions)


def score_directions(
    start_point: np.ndarray,
    directions: np.ndarray,
    u: np.ndarray,
    branch_center: np.ndarray,
    branch_orientation: np.ndarray,
    branch_radius: float,
    branch_length: float,
    lambda_dist: float = 0.35,  # Decay constant for distance
    sigma_fov_deg: Optional[float] = None,
    sigma_axis: float = 0.005,
) -> np.ndarray:
    sigma_fov = np.radians(sigma_fov_deg) if sigma_fov_deg is not None else lambda_dist
    scores = np.empty(shape=len(directions), dtype=float)

    # make axis unit vector
    axis = np.asarray(branch_orientation, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0:
        raise ValueError("branch_orientation is zero vector")
    axis_unit = axis / axis_norm

    # quadratic coefficients for a straight line Q(u) = 0*u^2 + (axis_unit)*u + branch_center
    A = np.zeros(3)  # second-degree term -> zero for straight line
    B = axis_unit  # first-degree term -> direction of axis (u in meters)
    C = branch_center  # base point (choose midpoint or endpoint)

    # coefs in the format expected: columns [A, B, C] -> shape (3,3)
    coefs = np.column_stack([A, B, C])  # shape (3,3)

    # choose u bounds: if C is midpoint => centered bounds
    u_min = -0.5 * branch_length
    u_max = 0.5 * branch_length

    for i, d in enumerate(directions):
        # ensure direction unit
        D = np.asarray(d, dtype=float)
        D_norm = np.linalg.norm(D)
        if D_norm == 0:
            scores[i] = 0.0
            continue
        D = D / D_norm

        hit_cyl, t_closest, u_hit = rt.ray_hits_quadratic(
            start_point=start_point, direction=D, branch_radius=branch_radius, coefs=coefs, u_min=u_min, u_max=u_max
        )

        if (not hit_cyl) or (t_closest < 0):
            scores[i] = 0.0
            continue

        # distance weight (closer intersection along the ray is better)
        distance_weight = np.exp(-t_closest / lambda_dist)

        # perpendicularity / FOV style term (favor perpendicular rays)
        # compute local tangent Q'(u) = 2*A*u + B
        tangent = 2.0 * A * u_hit + B
        tnorm = np.linalg.norm(tangent)
        if tnorm == 0:
            perp_weight = 0.0
        else:
            tangent_unit = tangent / tnorm
            perp_weight = 1.0 - abs(np.dot(D, tangent_unit))

        # axis_weight replacements:
        axis_weight = 1.0
        # if sigma_u is not None:
        #     # gaussian about the segment center: prefer hits near the middle of the fitted segment
        #     du = u_hit - u_center
        #     axis_weight *= np.exp(-0.5 * (du / sigma_u) ** 2)

        # if sigma_kappa is not None:
        #     # curvature kappa ≈ ||Q' x Q''|| / ||Q'||^3
        #     # Q''(u) = 2*A (constant)
        #     Qp = tangent
        #     Qpp = 2.0 * A
        #     cross = np.cross(Qp, Qpp)
        #     denom = np.linalg.norm(Qp) ** 3
        #     if denom <= 0:
        #         kappa = 0.0
        #     else:
        #         kappa = np.linalg.norm(cross) / denom
        #     axis_weight *= np.exp(-0.5 * (kappa / sigma_kappa) ** 2)

        # final score
        scores[i] = axis_weight * perp_weight * distance_weight

    return scores


def plot_cylindrical_start_points(
    r_range: ArrayLike,
    theta_range: ArrayLike,
    z_range: ArrayLike,
    num_r_pts: int,
    num_theta_pts: int,
    num_z_pts: int,
    start_point: np.ndarray,
    start_orientation: np.ndarray,
    branch_center: np.ndarray,
    branch_orientation: np.ndarray,
    branch_radius: float,
    branch_length: float,
    fig: go.Figure = None,
) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    points_global = generate_cylindrical_pts(
        r_range=r_range,
        theta_range=theta_range,
        z_range=z_range,
        num_r_pts=num_r_pts,
        num_theta_pts=num_theta_pts,
        num_z_pts=num_z_pts,
        start_point=start_point,
        start_orientation=start_orientation,
    )

    # Distances to branch
    vecs_pts_to_branch_pt = points_global - branch_center
    cross_ps = np.cross(vecs_pts_to_branch_pt, branch_orientation)
    distances = np.linalg.norm(cross_ps, axis=1) / np.linalg.norm(branch_orientation)

    branch_plane_normal = compute_plane_from_pts_and_orientation(
        p0=branch_center, p1=start_point, v0=branch_orientation
    )

    u = branch_center - start_point
    u /= np.linalg.norm(u)
    v = np.cross(u, branch_plane_normal)

    sensor_fov_deg = 18
    sigma_deg = sensor_fov_deg / 3

    generated_point_scores = np.zeros(shape=len(points_global), dtype=float)

    for i, pt in enumerate(points_global):
        if i % 100 == 0:
            print(f"Percentage done: {i/len(points_global) * 100}", end="\r")
        rotated_fov_pts = generate_cylindrical_pts(
            r_range=(0.04891, 0.04891),
            theta_range=(0, 2 * np.pi),
            z_range=(0, 0),
            num_r_pts=1,
            num_theta_pts=30,
            num_z_pts=1,
            start_point=pt,
            start_orientation=u,
        )

        rotated_pt_scores = np.zeros(shape=len(rotated_fov_pts), dtype=float)

        for j, rotated_pt in enumerate(rotated_fov_pts):
            sampled_directions = sample_gaussian_cone(
                u=u, v=v, w=branch_plane_normal, sensor_fov_deg=sensor_fov_deg, sigma_deg=sigma_deg, num_samples=1000
            )

            scored_directions = score_directions(
                start_point=rotated_pt,
                directions=sampled_directions,
                u=u,
                branch_center=branch_center,
                branch_orientation=branch_orientation,
                branch_radius=branch_radius,
                branch_length=branch_length,
                lambda_dist=0.25,
                sigma_fov_deg=sigma_deg,
                sigma_axis=0.01,
            )

            rotated_pt_scores[j] = np.mean(scored_directions)

        generated_point_scores[i] = np.mean(rotated_pt_scores)

    generated_point_scores = np.sqrt(generated_point_scores)

    fig.add_trace(
        go.Scatter3d(
            x=points_global[:, 0],
            y=points_global[:, 1],
            z=points_global[:, 2],
            mode="markers",
            marker=dict(size=4, color=generated_point_scores, colorscale="matter_r", opacity=0.1, showscale=True),
            name="generated_pts",
            customdata=np.column_stack((generated_point_scores)).reshape(-1, 1),
            hovertemplate=("x: %{x}<br>" "y: %{y}<br>" "z: %{z}<br>" "score: %{customdata[0]}<br>" "<extra></extra>"),
        ),
    )

    fig.update_layout(scene=dict(aspectmode="data"))

    return fig


def generate_radial_pts(
    roll_range: ArrayLike,
    pitch_range: ArrayLike,
    yaw_range: ArrayLike,
    z_range: ArrayLike,
    num_roll_pts: int,
    num_pitch_pts: int,
    num_yaw_pts: int,
    num_z_pts: int,
    # start_point: np.ndarray,
    # start_orientation: np.ndarray,
    branch_center: np.ndarray,
    branch_orientation: np.ndarray,
    # branch_radius: np.ndarray,
    # branch_length: np.ndarray,
):

    rolls = np.linspace(*roll_range, num_roll_pts)
    pitches = np.linspace(*pitch_range, num_pitch_pts)
    yaws = np.linspace(*yaw_range, num_yaw_pts)
    zs = np.linspace(*z_range, num_z_pts)

    rpy_grid = list(itertools.product(rolls, pitches, yaws))

    rotations = Rotation.from_euler("zyx", [(y, p, r) for r, p, y in rpy_grid])

    start_points = np.zeros(shape=(len(zs), 3))
    start_points[:, 2] = zs
    rotated_points = []
    for start_point in start_points:
        rotated_points.extend(rotations.apply(start_point))

    xyz = np.asarray(rotated_points)

    z_axis = np.array([0, 0, 1])
    orientation = branch_orientation / np.linalg.norm(branch_orientation)

    v = np.cross(z_axis, orientation)
    v /= np.linalg.norm(v)
    z_new = np.cross(orientation, v)

    rot_mat = np.column_stack((orientation, v, z_new))
    r = Rotation.from_matrix(rot_mat)
    points_global = r.apply(xyz) + branch_center

    # fig = go.Figure()
    # fig = ph.plot_vector(
    #     fig=fig, position=branch_center, orientation=orientation, scale=0.1, color="red", name="x", showlegend=True
    # )
    # fig = ph.plot_vector(
    #     fig=fig, position=branch_center, orientation=v, scale=0.1, color="green", name="y", showlegend=True
    # )
    # fig = ph.plot_vector(
    #     fig=fig, position=branch_center, orientation=z_new, scale=0.1, color="blue", name="z", showlegend=True
    # )
    # fig.add_trace(go.Scatter3d(x=points_global[:, 0], y=points_global[:, 1], z=points_global[:, 2], mode="markers"))

    # fig.update_layout(scene=dict(aspectmode="data"))
    # fig.show()

    # points_global = points_global[::6]

    # points_global = points_global[0:6]

    print(len(points_global))

    return points_global


def plot_radial_start_points(
    roll_range: ArrayLike,
    pitch_range: ArrayLike,
    yaw_range: ArrayLike,
    z_range: ArrayLike,
    num_roll_pts: int,
    num_pitch_pts: int,
    num_yaw_pts: int,
    num_z_pts: int,
    start_point: np.ndarray,
    start_orientation: np.ndarray,
    branch_center: np.ndarray,
    branch_orientation: np.ndarray,
    branch_radius: float,
    branch_length: float,
    fig: go.Figure = None,
):
    if fig is None:
        fig = go.Figure()

    points_global = generate_radial_pts(
        roll_range=roll_range,
        pitch_range=pitch_range,
        yaw_range=yaw_range,
        z_range=z_range,
        num_roll_pts=num_roll_pts,
        num_pitch_pts=num_pitch_pts,
        num_yaw_pts=num_yaw_pts,
        num_z_pts=num_z_pts,
        branch_center=branch_center,
        branch_orientation=branch_orientation,
    )

    fig = go.Figure()
    branch_orientation /= np.linalg.norm(branch_orientation)
    yaw_vals = np.linspace(start=yaw_range[0], stop=yaw_range[1], num=num_yaw_pts)
    yaw_vals /= 2

    generated_point_scores = np.zeros(shape=len(points_global), dtype=float)

    for i, point in enumerate(points_global):
        new_z = branch_center - point
        new_z /= np.linalg.norm(new_z)
        x_axis = np.cross([0, -1, 0], new_z)
        y_axis = np.cross(new_z, x_axis)
        r = np.column_stack((x_axis, y_axis, new_z))

        # yaw_orient = np.array([[np.cos(yaw_vals[i]), -np.sin(yaw_vals[i]), 0],
        #                        [np.sin(yaw_vals[i]), np.cos(yaw_vals[i]), 0],
        #                        [0, 0, 1]])

        # orientation_mat = r @ yaw_orient
        # orientation = np.diag(orientation_mat)
        # fig = ph.plot_3d_coordinate_frame(
        #     fig=fig,
        #     position=point,
        #     orientation=orientation_mat,
        #     axis_length=0.01,
        #     cone_scale=0.25,
        #     showlegend=False
        # )

        branch_plane_normal = compute_plane_from_pts_and_orientation(p0=branch_center, p1=point, v0=branch_orientation)
        branch_plane_normal /= np.linalg.norm(branch_plane_normal)

        eef_orientation = branch_center - point
        eef_orientation /= np.linalg.norm(eef_orientation)

        # fig = ph.plot_plane_from_point_and_normal_vec(
        #     point=point,
        #     norm=branch_plane_normal,
        #     fig=fig
        # )

        sensor_fov_deg = 18
        sigma_deg = sensor_fov_deg / 3
        v = np.cross(eef_orientation, branch_plane_normal)
        sampled_directions = sample_gaussian_cone(
            u=eef_orientation,
            v=v,
            w=branch_plane_normal,
            sensor_fov_deg=sensor_fov_deg,
            sigma_deg=sigma_deg,
            num_samples=1000,
        )

        # for direction in sampled_directions:
        #     fig.add_trace(
        #         go.Scatter3d(
        #             x=[point[0], point[0] + 0.1 * direction[0]],
        #             y=[point[1], point[1] + 0.1 * direction[1]],
        #             z=[point[2], point[2] + 0.1 * direction[2]],
        #             mode='lines',
        #             showlegend=False
        #         )
        #     )

        # print(sampled_directions)

        scored_directions = score_directions(
            start_point=point,
            directions=sampled_directions,
            u=eef_orientation,
            branch_center=branch_center,
            branch_orientation=branch_orientation,
            branch_radius=branch_radius,
            branch_length=branch_length,
            lambda_dist=0.30,
            sigma_fov_deg=sigma_deg,
            sigma_axis=0.01,
        )

        generated_point_scores[i] = np.mean(scored_directions)

    fig.add_trace(
        go.Scatter3d(
            x=points_global[:, 0],
            y=points_global[:, 1],
            z=points_global[:, 2],
            mode="markers",
            marker=dict(size=4, color=generated_point_scores, colorscale="matter_r", opacity=0.4, showscale=True),
            name="generated_pts",
            customdata=np.column_stack((generated_point_scores)).reshape(-1, 1),
            hovertemplate=("x: %{x}<br>" "y: %{y}<br>" "z: %{z}<br>" "score: %{customdata[0]}<br>" "<extra></extra>"),
        )
    )

    v = np.cross(branch_orientation, [1, 0, 0])
    fig.add_trace(
        go.Scatter3d(
            x=[branch_center[0] + v[0] * (branch_radius + 0.002)],
            y=[branch_center[1] + v[1] * (branch_radius + 0.002)],
            z=[branch_center[2] + v[2] * (branch_radius + 0.002)],
            name="branch_center",
            mode="markers",
            marker=dict(color="orange", opacity=1.0),
        )
    )

    fig.update_layout(scene=dict(aspectmode="data"))

    return fig


def main():

    return


if __name__ == "__main__":
    main()
