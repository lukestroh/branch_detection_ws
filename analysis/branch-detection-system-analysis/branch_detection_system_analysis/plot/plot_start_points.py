#!/usr/bin/env python3
import branch_detection_system_analysis.plot.plotly_helpers as ph


import plotly.graph_objects as go
import numpy as np
from numpy.typing import ArrayLike

from typing import Optional

def generate_cylindrical_pts(r_range: ArrayLike,
    theta_range: ArrayLike,
    z_range: ArrayLike,
    num_r_pts: int,
    num_theta_pts: int,
    num_z_pts: int,
    start_point: np.ndarray,
    start_orientation: np.ndarray
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
    sigma_deg = sensor_fov_deg/3

    generated_point_scores = np.zeros(shape=len(points_global), dtype=float)

    for i, pt in enumerate(points_global):
        rotated_fov_pts = generate_cylindrical_pts(
            r_range=(0.04891, 0.04891),
            theta_range=(0, 2*np.pi),
            z_range=(0,0),
            num_r_pts=1,
            num_theta_pts=30,
            num_z_pts=1,
            start_point=pt,
            start_orientation=u,
        )
        
        rotated_pt_scores = np.zeros(shape=len(rotated_fov_pts), dtype=float)

        for rotated_pt in rotated_fov_pts:
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
                lambda_dist=0.30,
                sigma_deg=sigma_deg
            )

        # generated_point_scores[i] = np.mean(scored_directions)


    fig.add_trace(
        go.Scatter3d(
            x=points_global[:, 0],
            y=points_global[:, 1],
            z=points_global[:, 2],
            mode="markers",
            marker=dict(size=5, color=generated_point_scores, colorscale="matter_r", showscale=False),
            name='generated_pts',
            customdata=np.column_stack((generated_point_scores)).reshape(-1, 1),
            hovertemplate=(
                "x: %{x}<br>"
                "y: %{y}<br>"
                "z: %{z}<br>"
                "score: %{customdata[0]}<br>"
                "<extra></extra>"
            ),
        ),
    )

    fig.update_layout(
        scene=dict(aspectmode='data')
    )

    return fig


# def ray_hits_cylinder(
#     start_point: np.ndarray,
#     direction: np.ndarray,
#     branch_center: np.ndarray,
#     branch_orientation: np.ndarray,
#     branch_radius: float,
#     branch_length: float,
# ):
#     """
#     Steps:
#     ------
#     1. Project the vector from the ray origin to the cylinder center onto the ray: t_closest = dot(v, direction).
#     2. Compute the point P = start_point + t_closest * direction.
#     3. Find the projection of P onto the cylinder axis: Q = branch_center + dot(P - branch_center, branch_orientation) * branch_orientation.
#     4. Compute the perpendicular distance: dist_perp = ||P - Q||.
#     5. The ray "hits" if dist_perp <= radius AND the axial coordinate z = dot(Q - branch_center, branch_orientation) lies within [-half_length, half_length].
#     """
#     # 1. Vector from ray origin to cylinder center
#     v = branch_center - start_point
#     # 2. Closest approach along the ray
#     t_closest = float(np.dot(v, direction))
#     # 3. Point on the ray at t_closest
#     P = start_point + t_closest * direction
#     # 4. Project P onto cylinder axis
#     axis_proj_length = np.dot(P - branch_center, branch_orientation)
#     Q = branch_center + axis_proj_length * branch_orientation
#     # 5. Perpendicular distance from ray to axis
#     dist_perp = np.linalg.norm(P - Q)
#     # 6. Check within radius and finite length
#     within_radius = dist_perp <= branch_radius
#     print(within_radius)
#     within_height = abs(axis_proj_length) <= branch_length / 2

#     hit = within_radius and within_height

#     return hit, t_closest


def ray_hits_cylinder(
    start_point: np.ndarray,
    direction: np.ndarray,
    branch_center: np.ndarray,
    branch_orientation: np.ndarray,
    branch_radius: float,
    branch_length: float
) -> tuple[bool, float]:
    # Normalize inputs
    direction = direction / np.linalg.norm(direction)
    branch_orientation = branch_orientation / np.linalg.norm(branch_orientation)
    # Line 1 (ray): P(t) = start_point + t * direction
    # Line 2 (cylinder axis): Q(s) = branch_center + s * branch_orientation
    w0 = start_point - branch_center
    a = np.dot(direction, direction)
    b = np.dot(direction, branch_orientation)
    c = np.dot(branch_orientation, branch_orientation)
    d = np.dot(direction, w0)
    e = np.dot(branch_orientation, w0)

    denom = a * c - b * b
    if np.isclose(denom, 0.0):
        return False, -1.0  # Lines are parallel; no unique closest point

    t = (b * e - c * d) / denom
    s = (a * e - b * d) / denom

    if t < 0:
        return False, -1.0  # Intersection point is behind the ray origin

    # Compute closest points on both lines
    closest_point_on_ray = start_point + t * direction
    closest_point_on_axis = branch_center + s * branch_orientation

    # Distance from ray to axis
    dist_perp = np.linalg.norm(closest_point_on_ray - closest_point_on_axis)

    # Check axial bounds of the finite cylinder
    if abs(s) > branch_length / 2:
        return False, t

    hit = dist_perp <= branch_radius

    return hit, t

def sample_gaussian_cone(
    u: np.ndarray,
    v: np.ndarray,
    w: np.ndarray,
    sensor_fov_deg: float,
    sigma_deg: float,
    num_samples: int,
    seed: int | None = None
):
    # Convert to rad
    sensor_fov = np.radians(sensor_fov_deg)
    sigma = np.radians(sigma_deg)
    max_radius = np.tan(sensor_fov/2)

    rgen = np.random.default_rng(seed=seed)

    sampled_directions = []
    while len(sampled_directions) < num_samples:
        # 2D Gaussian on tangent plane
        x, y = rgen.normal(loc=0, scale=sigma/3, size=2)
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
    lambda_dist: float = 0.35, # Decay constant for distance
    sigma_deg: Optional[float] = None
) -> np.ndarray:
    sigma = np.radians(sigma_deg) if sigma_deg is not None else lambda_dist

    scores = np.empty(shape=len(directions), dtype=float)

    for i, d in enumerate(directions):
        hit_cyl, t_closest = ray_hits_cylinder(
            start_point=start_point,
            direction=d,
            branch_center=branch_center,
            branch_orientation=branch_orientation,
            branch_radius=branch_radius,
            branch_length=branch_length,
        )

        if not hit_cyl or t_closest < 0.0:
            scores[i] = 0.0
            continue

        # Find angular weight
        cos_ratio = np.clip(np.dot(d, u), -1.0, 1.0)
        angle = np.arccos(cos_ratio)
        angular_weight = np.exp(-0.5 * (angle / sigma)**2)

        # Find distance weight
        distance_weight = np.exp(-t_closest / lambda_dist)

        scores[i] = angular_weight * distance_weight
    return scores



def main():

    return


if __name__ == "__main__":
    main()
