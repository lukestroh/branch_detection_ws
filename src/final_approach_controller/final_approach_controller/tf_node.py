#!/usr/bin/env python3
"""
tf_node.py
Adapted from https://github.com/OSUrobotics/follow-the-leader/blob/develop/follow_the_leader/follow_the_leader/utils/ros_utils.py. Improved generalizability (is that a word?).
Author(s): Alex You, Luke Strohbehn
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation
import numpy as np
import os
import yaml


class TFNode(Node):
    def __init__(self, node_name, *args, **kwargs) -> None:
        super().__init__(node_name=node_name, *args)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            # history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.tf_buffer = Buffer(cache_time=kwargs.get('cache_time'))
        self.tf_listener = TransformListener(self.tf_buffer, self, static_qos=qos)
        return

    def declare_parameter_dict(self, **kwargs):
        for key, val in kwargs.items():
            self.declare_parameter(key, val)
        return

    def get_param_val(self, key):
        try:
            return self.get_parameter(key).value
        except Exception as ex:
            self.get_logger().error(ex)
            return None

    def dump_params(self, dirname):
        name = self.get_name()
        yaml_output = {name: {"ros__parameters": {}}}
        for key, val in self.get_parameters_by_prefix("").items():
            yaml_output[name]["ros__parameters"][key] = val.value

        with open(os.path.join(dirname, f"params_{name}.yaml"), "w") as outfile:
            yaml.dump(yaml_output, outfile, default_flow_style=False)
        return

    def lookup_transform(
        self,
        target_frame,
        source_frame,
        time=None,
        sync=True,
        as_matrix=False,
        timeout=Duration(seconds=0.5),
    ):
        """Convenience function to lookup a transform

        :param target_frame: target
        :param source_frame: source
        :param time: time to use, defaults behaviour to use most recent transform
        :param sync: whether to use blocking sync, defaults to True.
        :param as_matrix: returns in homogenous matrix form, defaults to False
        :param timeout: how long to block, defaults to rclpy.time.Duration(seconds=0.5)
        :return: tf or matrix, None if failed
        """
        if time is None or not isinstance(time, rclpy.time.Time):
            time = rclpy.time.Time()
        start = self.get_clock().now()
        tf = None
        log_str = f"{self.get_name()}: TF lookup {source_frame} -> {target_frame}"
        try:
            if sync:
                tf = self.tf_buffer.lookup_transform(target_frame, source_frame, time, timeout=timeout)
            else:
                tf = self.tf_buffer.lookup_transform(target_frame, source_frame, time)
            if tf is None:
                raise TransformException("Likely timeout!")
        except TransformException as ex:
            self.get_logger().fatal(f"{log_str}: Received TF Exception: {ex}")
            return
        except Exception as ex:
            self.get_logger().fatal(f"{log_str}: Received Exception: {ex }")
            return
        wait = self.get_clock().now() - start
        if wait > rclpy.time.Duration(seconds=0.1):
            self.get_logger().warn(f"{log_str} took {wait.nanoseconds / 1e9} seconds")
        if not as_matrix:
            return tf

        tl = tf.transform.translation
        q = tf.transform.rotation
        mat = np.identity(4)
        mat[:3, 3] = [tl.x, tl.y, tl.z]
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        return mat
