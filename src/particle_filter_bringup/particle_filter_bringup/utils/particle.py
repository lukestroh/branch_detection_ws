#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Quaternion, Point, TransformStamped

import numpy as np

from typing import List

from rclpy.impl import rcutils_logger



class Particle:
    def __init__(self, tof_tfs: np.ndarray = None, tof_data: np.ndarray = None, camera_extrinsic: np.ndarray = None) -> None:
        """Defines a particle for the branch detection particle filter"""
        self.logger = rcutils_logger.RcutilsLogger(name="particle")

        # create homogenous vectors for ToF readings
        self.tof0_local_v = np.zeros((4,1))
        self.tof1_local_v = np.zeros((4,1))
        self.tof0_local_v[-1] = 1 
        self.tof1_local_v[-1] = 1

        # create output data vectors
        self.tof0_world = np.zeros((3,1))
        self.tof1_world = np.zeros((3,1))
        self.pose = np.zeros((6,1)) # 0:3 to the center of a branch, 3:6 is orientation

        self._get_initial_pose(tof_tfs, tof_data, camera_extrinsic)
        return
    
    def _get_initial_pose(self, tof_tfs: np.ndarray = None, tof_data: np.ndarray = None, camera_extrinsic: np.ndarray = None) -> None:
        """Get an output branch pose from the input poses
        
        Parameters
        ----------
            tof_tfs: np.ndarray
                -- A 2x4x4 array holding the TF matrices for each time of flight sensor to the world frame

        Returns
        -------
            None
        """
        if tof_tfs is None:
            tof_tfs = np.array([np.zeros((4,4)), np.zeros((4,4))])
        # Get the tof_to_world transforms
        tof0_to_world_tf = tof_tfs[0]
        tof1_to_world_tf = tof_tfs[1]

        # Update ToF sensor measurements as measurements in the +z direction in the ToF sensor coordinate frame
        if tof_data is None:
            self.tof0_local_v[2] = 0
            self.tof1_local_v[2] = 0
        else:
            self.tof0_local_v[2] = tof_data[0]
            self.tof1_local_v[2] = tof_data[1]

        # Get the two points in world frame
        point_tof0 = tof0_to_world_tf @ self.tof0_local_v
        point_tof1 = tof1_to_world_tf @ self.tof1_local_v

        self.tof0_world[:] = point_tof0[:3]
        self.tof1_world[:] = point_tof1[:3]
        
        center_world = (self.tof0_world + self.tof1_world) / 2
        orientation = self.tof0_world - self.tof1_world
        orientation = orientation / np.linalg.norm(orientation)
        

        self.pose[:3] = center_world
        self.pose[3:] = orientation

        return
    
    
    # def branch_line(self) -> np.ndarray:
    #     """Using the tof frames and the measured data, """
    #     return
    
    # Keep joint angles out of here, only put eef pose



    """We want to take into account the end-effector pose of the robot, a tof pose, and the camera data.

    Output should be either a line segment from point A to B, or a vector from the center of the two pointing upwards with respect to the world frame
    """


def main():
    rot = -1.0
    from rclpy.time import Time
    import tf2_ros
    from scipy.spatial.transform import Rotation
    _rot = Rotation.from_euler('x', np.pi).as_quat()

    tof_data = np.array([0.2564, 0.2612])

    tof0_tf = TransformStamped()
    tof0_tf.transform.translation.z = -0.9
    tof0_tf.transform.rotation.x = rot
    tof0_tf.header.stamp = Time(seconds=2.0, nanoseconds=0).to_msg()
    # tof0_tf.header.frame_id = 'a'
    # tof0_tf.child_frame_id = 'b' 

    tof1_tf = TransformStamped()
    tof1_tf.transform.translation.z = 0.1
    tof1_tf.transform.rotation.x = rot
    tof1_tf.header.stamp = Time(seconds=2.0, nanoseconds=0).to_msg()
    # tof1_tf.header.frame_id = 'a'
    # tof1_tf.child_frame_id = 'b'

    # tf_center_to_world = TransformStamped()
    # tf_center_to_world.transform.translation.y = -0.1
    # tf_center_to_world.transform.rotation.x = rot


    def tf_to_mat(tf):
        tl = tf.transform.translation
        q = tf.transform.rotation
        # return the 4x4 transformation matrix
        mat = np.identity(4)
        mat[:3, 3] = [tl.x, tl.y, tl.z]
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return mat  

    tof0_tf = tf_to_mat(tof0_tf)
    tof1_tf = tf_to_mat(tof1_tf)


    tof_tfs = [tof0_tf, tof1_tf]

    particle = Particle(tof_tfs=tof_tfs, tof_data=tof_data)
    
    print(particle.pose)
    return


if __name__ == "__main__":
    main()
