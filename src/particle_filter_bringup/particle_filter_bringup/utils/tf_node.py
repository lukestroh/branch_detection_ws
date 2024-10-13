#!/usr/bin/env python3
"""
tf_node.py
Adapted from https://github.com/OSUrobotics/follow-the-leader/blob/develop/follow_the_leader/follow_the_leader/utils/ros_utils.py. Improved generalizability (is that a word?).
Author(s): Alex You, Luke Strohbehn
"""

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation
import numpy as np

from threading import Event, Lock


class TFNode(Node):
    def __init__(self, node_name, *args, **kwargs) -> None:
        super().__init__(node_name=node_name, *args, **kwargs)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = self.declare_parameter("base_frame", value="base_link").get_parameter_value().string_value
        self.tool_frame = self.declare_parameter("tool_frame", value="tool0").get_parameter_value().string_value
        return

    def declare_parameter_dict(self, **kwargs):
        for key, val in kwargs.items():
            self._params[key] = self.declare_parameter(key, val)
        return

    def get_param_val(self, key):
        return self._params[key].value

    def lookup_transform(
        self, target_frame, source_frame, time=None, sync=True, as_matrix=False
    ):
        if time is None:
            time = rclpy.time.Time()
        if sync:
            future = self.tf_buffer.wait_for_transform_async(
                target_frame, source_frame, time
            )

            def wait_for_future_synced(future):
                event = Event()

                def done_callback(_):
                    nonlocal event
                    event.set()

                future.add_done_callback(done_callback)
                event.wait()
                resp = future.result()
                return resp

            wait_for_future_synced(future)

        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, time)
        if not as_matrix:
            return tf

        tl = tf.transform.translation
        q = tf.transform.rotation
        # return the 4x4 transformation matrix
        mat = np.identity(4)
        mat[:3, 3] = [tl.x, tl.y, tl.z]
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return mat
