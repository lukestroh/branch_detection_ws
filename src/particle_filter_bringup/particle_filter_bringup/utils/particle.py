#!/usr/bin/env python

from geometry_msgs.msg import Pose


class Particle:
    def __init__(self, ee_pose: Pose, tof_pose: Pose, camera_extrinsic: Pose) -> None:
        self.ee_pose: Pose = ee_pose
        # self.camera_extrinsic  -> tf frame?

        return

    """We want to take into account the end-effector pose of the robot, a tof pose, and the camera data.

    Output should be either a line segment from point A to B, or a vector from the center of the two pointing upwards with respect to the world frame
    """
