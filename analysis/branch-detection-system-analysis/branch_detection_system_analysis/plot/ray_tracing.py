#!/usr/bin/env python3
import numpy as np

import plotly.graph_objects as go


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


import numpy as np
import plotly.graph_objects as go


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
                continue
            # verify
            P = O + t * D
            Q = A * u**2 + B * u + C
            if abs(np.linalg.norm(P - Q) - branch_radius) < 1e-3:
                if t < t_hit:
                    t_hit, u_hit = t, u

    if u_hit is None:
        return False, -1, None
    return True, t_hit, u_hit


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
