#!/usr/bin/env python3
from __future__ import annotations
from nptyping import NDArray, Shape, Float
import numpy as np


def compute_perpendicular_projection_vector(ab: NDArray[Shape["3, 1"], Float], bc: NDArray[Shape["3, 1"], Float]):
    projection = ab - np.dot(ab, bc) / np.dot(bc, bc) * bc
    return projection


def get_dfov_from_fov(fov: tuple):
    return

def get_fov_from_dfov(camera_width, camera_height, dFoV):
    """
    Returns the vertical and horizontal field of view (FoV) in degrees given the diagonal field of view (dFoV) in degrees.
    https://www.litchiutilities.com/docs/fov.php
    https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix.html
    """
    _dfov = np.deg2rad(dFoV)
    camera_diag = np.sqrt(camera_width**2 + camera_height**2)
    fov_h = 2 * np.arctan(np.tan(_dfov / 2) * camera_height / camera_diag)
    fov_w = 2 * np.arctan(np.tan(_dfov / 2) * camera_width / camera_diag)
    return (np.rad2deg(fov_w), np.rad2deg(fov_h))



if __name__ == "__main__":
    camera_width = 64
    camera_height = 64
    dfov = 65

    fov = get_fov_from_dfov(camera_width, camera_height, dfov)
    print(fov)
