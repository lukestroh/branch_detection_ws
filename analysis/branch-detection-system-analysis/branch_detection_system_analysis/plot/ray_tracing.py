#!/usr/bin/env python3
import numpy as np
from numpy.typing import ArrayLike
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation

from typing import Optional

import branch_detection_system_analysis.plot.plotly_helpers as ph


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

    rotation = Rotation.align_vectors(orientation, z_axis)[0]
    rotated_pts = rotation.apply(xyz)

    points_global = rotated_pts + start_point

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


def ray_hits_quadratic_sampling(
    start_point: np.ndarray,
    direction: np.ndarray,
    coefs: np.ndarray,
    branch_radius: float,
    u_min: float = -np.inf,
    u_max: float = np.inf,
    num_samples: int = 200,
    refinement_iters: int = 10
) -> tuple[bool, Optional[float], Optional[float]]:
    """
    Find ray-cylinder intersection using sampling + Newton-Raphson refinement.
    
    This is more robust for small radii than the quartic root approach.
    
    Args:
        start_point: Ray origin (3D)
        direction: Ray direction (3D, will be normalized)
        coefs: 3x3 matrix where coefs[:, 0] = A, coefs[:, 1] = B, coefs[:, 2] = C
        branch_radius: Radius of cylinder
        u_min: Minimum curve parameter
        u_max: Maximum curve parameter
        num_samples: Number of samples along curve for initial search
        refinement_iters: Number of Newton-Raphson iterations
    
    Returns:
        (hit, t_hit, u_hit)
    """
    D = direction / np.linalg.norm(direction)
    O = start_point
    A, B, C = coefs[:, 0], coefs[:, 1], coefs[:, 2]
    
    if np.isinf(u_min):
        u_min = -10.0
    if np.isinf(u_max):
        u_max = 10.0
    
    # Sample the curve to find candidate regions
    u_samples = np.linspace(u_min, u_max, num_samples)
    candidates = []
    
    for u in u_samples:
        Q_u = A * u**2 + B * u + C
        
        # Find closest point on ray to Q_u
        W = Q_u - O
        t = W.dot(D)
        
        if t < 0:
            continue
        
        P = O + t * D
        dist = np.linalg.norm(P - Q_u)
        
        # If close to cylinder surface, mark as candidate
        if dist <= branch_radius * 1.5:  # Within 1.5x radius
            candidates.append((u, t, dist))
    
    if not candidates:
        return False, None, None
    
    # Refine each candidate using Newton-Raphson
    best_t = np.inf
    best_u = None
    
    for u_init, t_init, dist_init in candidates:
        u = u_init
        
        for _ in range(refinement_iters):
            # Curve point and derivatives
            Q_u = A * u**2 + B * u + C
            dQ_du = 2 * A * u + B
            d2Q_du2 = 2 * A
            
            # Closest point on ray
            W = Q_u - O
            t = W.dot(D)
            
            if t < 0:
                break
            
            P = O + t * D
            
            # Distance from ray to curve
            vec = P - Q_u
            dist = np.linalg.norm(vec)
            
            # Check if we found a hit
            if abs(dist - branch_radius) < 1e-6:
                if t < best_t:
                    best_t = t
                    best_u = u
                break
            
            # Newton step: minimize f(u) = ||P(u) - Q(u)||² - r²
            # where P(u) is closest ray point to Q(u)
            
            # Gradient of distance w.r.t. u
            # d(dist)/du at the closest point
            if dist > 1e-10:
                # dt/du = dQ/du · D
                dt_du = dQ_du.dot(D)
                
                # d(vec)/du = d(P-Q)/du = D * dt/du - dQ/du
                dvec_du = D * dt_du - dQ_du
                
                # d(dist)/du = (vec · dvec/du) / dist
                ddist_du = vec.dot(dvec_du) / dist
                
                # Newton step to find where dist = radius
                error = dist - branch_radius
                u_new = u - error / (ddist_du + 1e-10)
                
                # Clamp to valid range
                u_new = np.clip(u_new, u_min, u_max)
                
                if abs(u_new - u) < 1e-8:
                    break
                
                u = u_new
            else:
                break
        
        # Final check
        Q_u = A * u**2 + B * u + C
        W = Q_u - O
        t = W.dot(D)
        
        if t > 0:
            P = O + t * D
            dist = np.linalg.norm(P - Q_u)
            
            if abs(dist - branch_radius) < branch_radius * 0.1:  # Within 10% of radius
                if t < best_t:
                    best_t = t
                    best_u = u
    
    if best_u is None:
        return False, None, None
    
    return True, best_t, best_u

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
            print(f"u: {u}")
            u = float(np.real(u))
            if not (u_min <= u <= u_max):
                continue
            Mu = A * u**2 + B * u + C0
            t = D.dot(Mu)
            if t < 0:
                print("t: ", t)
                continue
            # verify
            P = O + t * D
            Q = A * u**2 + B * u + C
            print("DIFF: ", abs(np.linalg.norm(P - Q) - branch_radius))
            if np.isclose(abs(np.linalg.norm(P - Q) - branch_radius), 0.0065, atol=0.001):
                if t < t_hit:
                    t_hit, u_hit = t, u

    if u_hit is None:
        return False, None, None
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
    debug_fig: go.Figure = None,
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
            raise ValueError("D_norm is 0.0")
            scores[i] = 0.0
            continue
        D = D / D_norm

        # call the intersection routine
        hit, t_closest, u_hit = ray_hits_quadratic_sampling(
            start_point=start_point,
            direction=D,
            coefs=coefs,
            branch_radius=branch_radius,
            u_min=u_min,
            u_max=u_max,
            num_samples=500,
        )
        # print(t_closest)
        if (not hit) or (t_closest < -1) or (t_closest > 1):
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


def interpolate_vertex_normals(
    vertex_normals: np.ndarray,
    face: np.ndarray,
    barycentric: np.ndarray
) -> np.ndarray:
    """
    Interpolate vertex normals using barycentric coordinates.
    
    Args:
        vertex_normals: Per-vertex normals (N, 3)
        face: Triangle face (3 vertex indices)
        barycentric: Barycentric coordinates (3,)
    
    Returns:
        Interpolated normal vector
    """
    n0 = vertex_normals[face[0]]
    n1 = vertex_normals[face[1]]
    n2 = vertex_normals[face[2]]
    
    normal = (barycentric[0] * n0 + 
              barycentric[1] * n1 + 
              barycentric[2] * n2)
    
    norm = np.linalg.norm(normal)
    if norm > 1e-10:
        return normal / norm
    else:
        return normal
    

def compute_mesh_normal(
    vertices: np.ndarray,
    face: np.ndarray
) -> np.ndarray:
    """
    Compute face normal for a triangle.
    
    Args:
        vertices: Vertex array
        face: Triangle face (3 vertex indices)
    
    Returns:
        Normalized normal vector
    """
    v0 = vertices[face[0]]
    v1 = vertices[face[1]]
    v2 = vertices[face[2]]
    
    edge1 = v1 - v0
    edge2 = v2 - v0
    
    normal = np.cross(edge1, edge2)
    norm = np.linalg.norm(normal)
    
    if norm > 1e-10:
        return normal / norm
    else:
        return np.array([0.0, 0.0, 1.0])
    

def mesh_dict_to_arrays(mesh_dict: dict) -> tuple[np.ndarray, np.ndarray]:
    """
    Convert mesh dictionary format to vertices and faces arrays.
    
    Args:
        mesh_dict: Dictionary with keys 'x', 'y', 'z', 'i', 'j', 'k'
                   x, y, z: arrays of vertex coordinates
                   i, j, k: arrays of triangle vertex indices
    
    Returns:
        vertices: Array of shape (N, 3)
        faces: Array of shape (M, 3)
    """
    x = np.array(mesh_dict['x'])
    y = np.array(mesh_dict['y'])
    z = np.array(mesh_dict['z'])
    
    vertices = np.column_stack([x, y, z])
    
    i = np.array(mesh_dict['i'])
    j = np.array(mesh_dict['j'])
    k = np.array(mesh_dict['k'])
    
    faces = np.column_stack([i, j, k])
    
    return vertices, faces


def ray_triangle_intersection(
    ray_origin: np.ndarray,
    ray_direction: np.ndarray,
    v0: np.ndarray,
    v1: np.ndarray,
    v2: np.ndarray,
    epsilon: float = 1e-8
) -> Optional[tuple[np.ndarray, float, np.ndarray]]:
    """
    Möller-Trumbore ray-triangle intersection algorithm.
    
    Args:
        ray_origin: Ray starting point (3D)
        ray_direction: Ray direction (3D, normalized)
        v0, v1, v2: Triangle vertices (3D)
        epsilon: Numerical tolerance
    
    Returns:
        If hit: (intersection_point, distance, barycentric_coords)
        If miss: None
    """
    # Edge vectors
    edge1 = v1 - v0
    edge2 = v2 - v0
    
    # Begin calculating determinant
    h = np.cross(ray_direction, edge2)
    a = np.dot(edge1, h)
    
    # Ray is parallel to triangle
    if abs(a) < epsilon:
        return None
    
    f = 1.0 / a
    s = ray_origin - v0
    u = f * np.dot(s, h)
    
    # Intersection outside triangle
    if u < 0.0 or u > 1.0:
        return None
    
    q = np.cross(s, edge1)
    v = f * np.dot(ray_direction, q)
    
    # Intersection outside triangle
    if v < 0.0 or u + v > 1.0:
        return None
    
    # Calculate t (distance along ray)
    t = f * np.dot(edge2, q)
    
    # Ray intersection
    if t > epsilon:
        intersection_point = ray_origin + t * ray_direction
        barycentric = np.array([1.0 - u - v, u, v])
        return (intersection_point, t, barycentric)
    
    # Line intersection but not ray (behind origin)
    return None


def ray_mesh_intersection_any(
    ray_origin: np.ndarray,
    ray_direction: np.ndarray,
    vertices: np.ndarray,
    faces: np.ndarray
) -> bool:
    """
    Fast test if ray intersects mesh at all (doesn't compute intersection point).
    
    Args:
        ray_origin: Ray starting point (3D)
        ray_direction: Ray direction (3D)
        vertices: Vertex array of shape (N, 3)
        faces: Face array of shape (M, 3) with vertex indices
    
    Returns:
        True if any intersection exists, False otherwise
    """
    ray_direction = ray_direction / np.linalg.norm(ray_direction)
    
    for face in faces:
        v0 = vertices[face[0]]
        v1 = vertices[face[1]]
        v2 = vertices[face[2]]
        
        if ray_triangle_intersection(ray_origin, ray_direction, v0, v1, v2) is not None:
            return True
    
    return False


def ray_mesh_intersection(
    ray_origin: np.ndarray,
    ray_direction: np.ndarray,
    vertices: np.ndarray,
    faces: np.ndarray,
    return_all: bool = False
) -> Optional[tuple[np.ndarray, float, int, np.ndarray]]:
    """
    Test ray intersection with a triangular mesh.
    
    Args:
        ray_origin: Ray starting point (3D)
        ray_direction: Ray direction (3D, should be normalized)
        vertices: Vertex array of shape (N, 3)
        faces: Face array of shape (M, 3) with vertex indices
        return_all: If True, return all intersections; if False, return closest
    
    Returns:
        If return_all=False (default):
            (intersection_point, distance, face_index, barycentric_coords) or None
        If return_all=True:
            List of (intersection_point, distance, face_index, barycentric_coords)
    """
    ray_direction = ray_direction / np.linalg.norm(ray_direction)
    
    intersections = []
    
    for face_idx, face in enumerate(faces):
        v0 = vertices[face[0]]
        v1 = vertices[face[1]]
        v2 = vertices[face[2]]
        
        result = ray_triangle_intersection(ray_origin, ray_direction, v0, v1, v2)
        
        if result is not None:
            hit_point, distance, barycentric = result
            intersections.append((hit_point, distance, face_idx, barycentric))
    
    if not intersections:
        return None if not return_all else []
    
    if return_all:
        return intersections
    
    # Return closest intersection
    closest = min(intersections, key=lambda x: x[1])
    return closest


def find_closest_u_on_curve(
    point: np.ndarray,
    coefs: np.ndarray,
    u_min: float,
    u_max: float,
    num_samples: int = 100
) -> float:
    """
    Find curve parameter u where curve is closest to a point.
    Uses coarse sampling + refinement.
    """
    # Coarse sampling
    u_samples = np.linspace(u_min, u_max, num_samples)
    A, B, C = coefs[:, 0], coefs[:, 1], coefs[:, 2]
    
    min_dist = float('inf')
    best_u = u_min
    
    for u in u_samples:
        curve_point = A * u**2 + B * u + C
        dist = np.linalg.norm(curve_point - point)
        if dist < min_dist:
            min_dist = dist
            best_u = u
    
    # Gradient descent refinement
    u = best_u
    step_size = (u_max - u_min) / num_samples
    
    for _ in range(10):
        curve_point = A * u**2 + B * u + C
        to_point = point - curve_point
        current_dist = np.linalg.norm(to_point)
        
        if current_dist < 1e-10:
            break
        
        # Tangent vector (derivative)
        tangent = 2 * A * u + B
        
        # Gradient of distance with respect to u
        gradient = -np.dot(to_point, tangent) / (current_dist + 1e-10)
        
        # Update u
        u_new = u - step_size * gradient
        u_new = np.clip(u_new, u_min, u_max)
        
        new_curve_point = A * u_new**2 + B * u_new + C
        new_dist = np.linalg.norm(new_curve_point - point)
        
        if new_dist < current_dist:
            u = u_new
            step_size *= 1.2
        else:
            step_size *= 0.5
        
        step_size = min(step_size, (u_max - u_min) * 0.1)
    
    return float(np.clip(u, u_min, u_max))



def score_quadratic_directions2(
    start_point: np.ndarray,
    directions: np.ndarray,  # (N,3) array or list of direction vectors
    coefs: np.ndarray,  # 3x3 matrix: columns are [A, B, C]
    t_vals: np.ndarray,
    branch_radius: float,
    mesh_vertices: np.ndarray,
    mesh_faces: np.ndarray,
    tube_mesh_info: dict,
    u_min: float = -np.inf,
    u_max: float = np.inf,
    lambda_dist: float = 0.25,
    sigma_fov_deg: Optional[float] = None,
    # NEW options for axis_weight replacement:
    sigma_u: Optional[float] = None,  # gaussian width in u-space (if None, no u-central penalty)
    sigma_kappa: Optional[float] = None,  # width for curvature penalty (if None, no curvature penalty)
    debug_fig: go.Figure = None,
) -> np.ndarray:
        
    scores = np.zeros(len(directions), dtype=float)

    # pre-extract quadratic coefficients for derivative computations
    A = coefs[:, 0]
    B = coefs[:, 1]

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
        res = ray_mesh_intersection(ray_origin=start_point, ray_direction=D, vertices=mesh_vertices, faces=mesh_faces, return_all=False)
        # fig = go.Figure()
        # fig = ph.plot_tube_mesh(fig=fig, tube_mesh_info=tube_mesh_info)
        # fig.add_trace(
        #     go.Scatter3d(
        #         x=[start_point[0], start_point[0] + D[0]* 0.2],
        #         y=[start_point[1], start_point[1] + D[1]* 0.2],
        #         z=[start_point[2], start_point[2] + D[2]* 0.2],
        #     )
        # )
        # fig.update_layout(scene=dict(aspectmode='data'))
        # fig.show()
        if res:
            intersection_pt, distance, face_id, bary_coords = res
            # print(intersection_pt)
        else:
            continue
        
        t_closest = find_closest_t_on_curve(
            point=intersection_pt,
            coefs=coefs,
            t_min=np.min(t_vals), 
            t_max=np.max(t_vals),
            num_samples=len(t_vals),
            refine_iterations=5
        )
        u_hit = find_closest_u_on_curve(
            point=intersection_pt,
            coefs=coefs,
            u_min=u_min,
            u_max=u_max
        )

        

        # print(f"{i}: {intersection_pt}, closest t: {t_closest}")

        if intersection_pt is None or t_closest is None:
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


def quadratic_curve(t: float, coefs: np.ndarray) -> np.ndarray:
    return coefs[:, 0] * t**2 + coefs[:, 1] * t + coefs[:, 2]


def find_closest_t_on_curve(
    point: np.ndarray,
    coefs: np.ndarray,
    t_min: float,
    t_max: float,
    num_samples: int = 50,
    refine_iterations: int = 5
) -> float:
    """
    Find the curve parameter t where the curve is closest to a given point.
    
    Uses a coarse sampling followed by gradient descent refinement.
    
    Args:
        point: 3D point to find closest curve parameter for
        coeffs: 3x3 coefficient matrix for quadratic curve
        t_min: Minimum t value
        t_max: Maximum t value
        num_samples: Number of initial samples
        refine_iterations: Number of refinement iterations
    
    Returns:
        Parameter t where curve is closest to point
    """
    # Coarse sampling to find approximate closest point
    t_samples = np.linspace(t_min, t_max, num_samples)
    min_dist = float('inf')
    best_t = t_min
    
    for t in t_samples:
        curve_point = quadratic_curve(t, coefs)
        dist = np.linalg.norm(curve_point - point)
        if dist < min_dist:
            min_dist = dist
            best_t = t
    
    # Refine using gradient descent
    t = best_t
    step_size = (t_max - t_min) / num_samples
    
    for _ in range(refine_iterations):
        # Current distance
        curve_point = quadratic_curve(t, coefs)
        to_point = point - curve_point
        current_dist = np.linalg.norm(to_point)
        
        if current_dist < 1e-10:
            break
        
        # Tangent vector (derivative)
        tangent = 2 * coefs[:, 0] * t + coefs[:, 1]
        
        # Gradient of distance squared with respect to t
        # d/dt ||C(t) - P||^2 = 2 * (C(t) - P) · C'(t)
        gradient = -2 * np.dot(to_point, tangent)
        
        # Update t (gradient descent)
        t_new = t - step_size * gradient / (abs(gradient) + 1e-10)
        
        # Clamp to valid range
        t_new = np.clip(t_new, t_min, t_max)
        
        # Check if we're improving
        new_curve_point = quadratic_curve(t_new, coefs)
        new_dist = np.linalg.norm(new_curve_point - point)
        
        if new_dist < current_dist:
            t = t_new
            step_size *= 1.2  # Increase step if improving
        else:
            step_size *= 0.5  # Decrease step if not improving
        
        # Adaptive step size limit
        step_size = min(step_size, (t_max - t_min) * 0.1)
    
    return float(np.clip(t, t_min, t_max))


def compute_perpendicularity(ray_direction: np.ndarray, surface_normal: np.ndarray) -> float:
    """
    Compute perpendicularity score [0, 1].
    1.0 = perpendicular (head-on), 0.0 = parallel (grazing)
    """
    return abs(np.dot(ray_direction, surface_normal))



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


# if __name__ == "__main__":
#     main()


# Example usage
if __name__ == "__main__":
    # import branch_detection_system_analysis.plot.plotly_helpers as ph
    import os
    import h5py as hpy
    import pandas as pd

    __here__ = os.path.dirname(__file__)
    
    
    coefs = np.array(hpy.File(f"{__here__}/data/coefs.h5").get("coefs"))

    quadratic_curve_df = pd.read_hdf(f"{__here__}/data/quadratic_curve.h5")
    t_vals = quadratic_curve_df['t_vals'].to_numpy()
    plot_t_vals = np.linspace(sorted(t_vals)[0], sorted(t_vals)[-1], len(t_vals))
    u_min, u_max = t_vals.min() - 1e-6, t_vals.max() + 1e-6

    tube_mesh_info = ph.get_tube_mesh_info(coefs=coefs, radius=0.0065, u_vals=plot_t_vals)
    
    ray_origin = np.array([-0.33600560496440923, 1.0701252570127726, 1.7842487132420677])
    ray_direction = np.array([ 0.1942305, 0.97457098, -0.11174039])

    vertices, faces = mesh_dict_to_arrays(mesh_dict=tube_mesh_info)

    # Basic intersection
    result = ray_mesh_intersection(ray_origin, ray_direction, vertices=vertices, faces=faces)
    print(result)


    fig = go.Figure()
    fig = ph.plot_tube_mesh(fig=fig, tube_mesh_info=tube_mesh_info)
    fig.add_trace(
        go.Scatter3d(
            x=[ray_origin[0], ray_origin[0] + ray_direction[0]* 0.2],
            y=[ray_origin[1], ray_origin[1] + ray_direction[1]* 0.2],
            z=[ray_origin[2], ray_origin[2] + ray_direction[2]* 0.2],
        )
    )
    fig.update_layout(scene=dict(aspectmode='data'))
    fig.show()


    # if result:
    #     print(f"Basic hit: distance={result[1]:.4f}")
    
    # if result:
    #     print(f"Curve-scored hit:")
    #     print(f"  u_hit: {result['u_hit']:.4f}")
    #     print(f"  perp_weight: {result['perp_weight']:.4f}")
    #     print(f"  axis_weight: {result['axis_weight']:.4f}")