#!/usr/bin/env python3
import numpy as np
from numpy.typing import ArrayLike
import plotly.graph_objects as go

from typing import Optional


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


def ray_hits_cylinder(
    start_point: np.ndarray,
    direction: np.ndarray,
    branch_center: np.ndarray,
    branch_orientation: np.ndarray,
    branch_radius: float,
    branch_length: float,
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


def ray_hits_quadratic(start_point, direction, coefs, branch_radius, u_min=-np.inf, u_max=np.inf):
    D = direction / np.linalg.norm(direction)
    O = start_point
    A, B, C = coefs[:, 0], coefs[:, 1], coefs[:, 2]
    C0 = C - O

    # build quartic coefficients
    m4 = A.dot(A)
    m3 = 2 * A.dot(B)
    m2 = 2 * A.dot(C0) + B.dot(B)
    m1 = 2 * B.dot(C0)
    m0 = C0.dot(C0)

    a2, a1, a0 = D.dot(A), D.dot(B), D.dot(C0)
    d4 = a2**2
    d3 = 2 * a2 * a1
    d2 = 2 * a2 * a0 + a1**2
    d1 = 2 * a1 * a0
    d0 = a0**2

    poly = np.array(
        [
            m4 - d4,
            m3 - d3,
            m2 - d2,
            m1 - d1,
            m0 - d0 - branch_radius**2,
        ]
    )

    roots = np.roots(poly)
    t_hit, u_hit = np.inf, None
    for u in roots:
        if np.isreal(u):
            u = float(np.real(u))
            if not (u_min <= u <= u_max):
                continue
            Mu = A * u**2 + B * u + C0
            t = D.dot(Mu)
            if t < 0:
                print(t)
                continue
            # verify
            P = O + t * D
            Q = A * u**2 + B * u + C
            if abs(np.linalg.norm(P - Q) - branch_radius) < 0.0065 * 0.01:
                if t < t_hit:
                    t_hit, u_hit = t, u

    if u_hit is None:
        return False, -1, None
    return True, t_hit, u_hit


def score_quadratic_directions(
    start_point: np.ndarray,
    directions: np.ndarray,  # (N,3) array or list of direction vectors
    coefs: np.ndarray,  # 3x3 matrix: columns are [A, B, C]
    branch_radius: float,
    u_min: float = -np.inf,
    u_max: float = np.inf,
    lambda_dist: float = 0.25,
    sigma_fov_deg: Optional[float] = None,
    # NEW options for axis_weight replacement:
    sigma_u: Optional[float] = None,  # gaussian width in u-space (if None, no u-central penalty)
    sigma_kappa: Optional[float] = None,  # width for curvature penalty (if None, no curvature penalty)
) -> np.ndarray:
    """
    Score directions for hitting a quadratic tube.

    Returns scores array of shape (len(directions),).

    Notes:
      - Uses ray_hits_quadratic(start_point, direction, coefs, branch_radius, u_min, u_max)
        which must return (hit:bool, t:float, u:float).
      - axis_weight is either a Gaussian in u (if sigma_u provided) or a Gaussian on curvature
        (if sigma_kappa provided); if both provided, they are multiplied together.
    """
    # prepare
    sigma_fov = np.radians(sigma_fov_deg) if sigma_fov_deg is not None else None
    scores = np.zeros(len(directions), dtype=float)

    # pre-extract quadratic coefficients for derivative computations
    A = coefs[:, 0]
    B = coefs[:, 1]
    # C = coefs[:, 2]  # not needed for tangent/curvature

    # u center for u-based weight
    if np.isfinite(u_min) and np.isfinite(u_max):
        u_center = 0.5 * (u_min + u_max)
        if sigma_u is None:
            # default sigma_u as half the segment length in u space (so ends are ~exp(-0.5) away)
            sigma_u = max(1e-6, 0.5 * (u_max - u_min))
    else:
        u_center = 0.0
        # if sigma_u given keep it; otherwise leave None (no u penalty)

    for i, d in enumerate(directions):
        # ensure direction unit
        D = np.asarray(d, dtype=float)
        D_norm = np.linalg.norm(D)
        if D_norm == 0:
            scores[i] = 0.0
            continue
        D = D / D_norm

        # call the intersection routine
        hit, t_closest, u_hit = ray_hits_quadratic(
            start_point=start_point,
            direction=D,
            coefs=coefs,
            branch_radius=branch_radius,
            u_min=u_min,
            u_max=u_max,
        )

        if (not hit) or (t_closest < 0):
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
        if sigma_u is not None:
            # gaussian about the segment center: prefer hits near the middle of the fitted segment
            du = u_hit - u_center
            axis_weight *= np.exp(-0.5 * (du / sigma_u) ** 2)

        if sigma_kappa is not None:
            # curvature kappa ≈ ||Q' x Q''|| / ||Q'||^3
            # Q''(u) = 2*A (constant)
            Qp = tangent
            Qpp = 2.0 * A
            cross = np.cross(Qp, Qpp)
            denom = np.linalg.norm(Qp) ** 3
            if denom <= 0:
                kappa = 0.0
            else:
                kappa = np.linalg.norm(cross) / denom
            axis_weight *= np.exp(-0.5 * (kappa / sigma_kappa) ** 2)

        # final score
        scores[i] = axis_weight * perp_weight * distance_weight

    return scores


def make_tube_mesh(coefs, radius, u_vals, n_theta=16):
    # compute curve centers at each u
    centers = (coefs @ np.vstack([u_vals**2, u_vals, np.ones_like(u_vals)])).T  # (len(u_vals),3)

    # tangents Q'(u) = 2*A*u + B
    tgs = 2 * coefs[:, 0] * u_vals[:, None] + coefs[:, 1]
    tgs = tgs / np.linalg.norm(tgs, axis=1)[:, None]

    # create normals and binormals
    up = np.array([0, 0, 1.0])
    normals = np.cross(tgs, up)
    mask = np.linalg.norm(normals, axis=1) < 1e-6
    if mask.any():
        up2 = np.array([0, 1, 0])
        normals[mask] = np.cross(tgs[mask], up2)
    normals = normals / np.linalg.norm(normals, axis=1)[:, None]
    binormals = np.cross(tgs, normals)

    # build mesh vertices
    X, Y, Z = [], [], []
    for Cpt, N, B in zip(centers, normals, binormals):
        for theta in np.linspace(0, 2 * np.pi, n_theta, endpoint=False):
            pt = Cpt + radius * (np.cos(theta) * N + np.sin(theta) * B)
            X.append(pt[0])
            Y.append(pt[1])
            Z.append(pt[2])

    # build triangle indices
    n_u = len(u_vals)
    I, J, K = [], [], []
    for i in range(n_u - 1):
        for j in range(n_theta):
            nj = (j + 1) % n_theta
            a = i * n_theta + j
            b = (i + 1) * n_theta + j
            c = i * n_theta + nj
            d = (i + 1) * n_theta + nj
            I += [a, c]
            J += [b, b]
            K += [d, d]

    return dict(x=X, y=Y, z=Z, i=I, j=J, k=K)


def main():
    # Define quadratic: x=u^2, y=u, z=0
    coefs = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]])
    radius = 0.2
    u_vals = np.linspace(-2, 2, 50)

    # test rays
    tests = [
        (np.array([1, 1, 1]), np.array([0, 0, -1])),  # hit
        (np.array([2, 1, 1]), np.array([0, 0, -1])),  # miss
    ]

    fig = go.Figure()
    # add tube mesh
    mesh = make_tube_mesh(coefs, radius, u_vals, n_theta=24)
    fig.add_trace(go.Mesh3d(**mesh, opacity=0.3, name="Tube"))

    # add curve centerline
    curve = np.vstack(
        [
            coefs[0, 0] * u_vals**2 + coefs[0, 1] * u_vals + coefs[0, 2],
            coefs[1, 0] * u_vals**2 + coefs[1, 1] * u_vals + coefs[1, 2],
            np.zeros_like(u_vals),
        ]
    )
    fig.add_trace(go.Scatter3d(x=curve[0], y=curve[1], z=curve[2], mode="lines", name="Curve"))

    # add rays and hits
    colors = ["red", "blue"]
    for idx, (O, D) in enumerate(tests):
        hit, t, u = ray_hits_quadratic(O, D, coefs, radius)
        P_end = O + (t if hit else 5) * D / np.linalg.norm(D)
        fig.add_trace(
            go.Scatter3d(
                x=[O[0], P_end[0]],
                y=[O[1], P_end[1]],
                z=[O[2], P_end[2]],
                mode="lines",
                name=f"Ray {idx} hit={hit}",
                line=dict(color=colors[idx]),
            )
        )
        if hit:
            Phit = O + t * D / np.linalg.norm(D)
            fig.add_trace(
                go.Scatter3d(
                    x=[Phit[0]],
                    y=[Phit[1]],
                    z=[Phit[2]],
                    mode="markers+text",
                    text=[f"u={u:.2f}"],
                    textposition="top center",
                    marker=dict(size=4),
                    name=f"Hit {idx}",
                )
            )

    fig.update_layout(title="Ray-Tube Intersection: Quadratic Curve")
    fig.show()
    return


if __name__ == "__main__":
    main()
