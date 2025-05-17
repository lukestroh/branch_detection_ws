#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from ros2bag_msgs.srv import StartRecord, StopRecord
from std_msgs.msg import String

import datetime as dt
import subprocess
import os


class RecordBagNode(Node):
    def __init__(self):
        super().__init__(node_name="record_bag_node")
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.error = lambda x: self.get_logger().error(f"{x}")

        # Parameters
        self._param_record_bag = (
            self.declare_parameter(name="record_bag", value=Parameter.Type.BOOL).get_parameter_value().bool_value
        )
        self._param_record_loc = (
            self.declare_parameter(name="record_loc", value=Parameter.Type.STRING).get_parameter_value().string_value
        )

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()

        # Action servers

        # Service servers
        self._srv_server_start_bag_record = self.create_service(
            srv_name="start_bag_record",
            srv_type=StartRecord,
            callback=self._service_svr_cb_start_bag_record,
            callback_group=self._cb_group_reentrant,
        )

        self._srv_server_stop_bag_record = self.create_service(
            srv_name="stop_bag_record",
            srv_type=StopRecord,
            callback=self._service_srv_cb_stop_bag_record,
            callback_group=self._cb_group_reentrant,
        )

        # Publishers
        self._pub_bag_path = self.create_publisher(
            msg_type=String, topic="bag_record_path", qos_profile=1, callback_group=self._cb_group_reentrant
        )

        # Timers
        # self.debug_timer = self.create_timer(timer_period_sec=1.0, callback=self.debug_timer_cb)

        # Subprocesses
        self.bag_process: subprocess.Popen

        return

    def debug_timer_cb(self):
        start_req = StartRecord.Request()
        self.info(start_req.topics)
        return

    def _service_svr_cb_start_bag_record(self, request: StartRecord.Request, response: StartRecord.Response):
        # Subprocess requires relative path for bag creation
        if self._param_record_bag:
            cwd_path = os.getcwd()
            # _record_loc_str = request.record_loc # TODO: refactor away request info, just pass from launch not behavior
            _record_loc_str = self._param_record_loc

            if _record_loc_str != "":
                _record_loc = _record_loc_str + "__"
            else:
                _record_loc = _record_loc_str

            if request.output_dir != "":
                _filepath_bag = os.path.join(
                    os.path.expanduser(request.output_dir),
                    f"bds__{_record_loc}{dt.datetime.strftime(dt.datetime.now(), format=r'%Y%m%d_%H-%M-%S')}",
                )
            else:
                _filepath_bag = os.path.join(
                    os.path.expanduser("~"),
                    "branch_detection_ws",
                    "bags",
                    f"bds__{_record_loc}{dt.datetime.strftime(dt.datetime.now(), format=r'%Y%m%d_%H-%M-%S')}",
                )
            # Absolute path for the publisher
            abs_path = os.path.abspath(_filepath_bag)
            self._pub_bag_path.publish(msg=String(data=abs_path))

            # Relative path for the python subprocess
            rel_path = os.path.relpath(path=_filepath_bag, start=cwd_path)
            self.bag_process = subprocess.Popen(
                args=[
                    "ros2",
                    "bag",
                    "record",
                    *(request.topics),
                    f"--compression-mode",
                    request.compression_mode,
                    f"--compression-format",
                    request.compression_format,
                    f"--output",
                    rel_path,
                ],
                stderr=subprocess.PIPE,
                stdout=subprocess.PIPE,
            )
            response.success = True
        else:
            response.success = False
        return response

    def _service_srv_cb_stop_bag_record(self, request: StopRecord.Request, response: StopRecord.Response):
        self.bag_process.terminate()
        terminate_success = self.bag_process.wait()

        if terminate_success == 0:
            response.success = True
        else:
            response.success = False
        return response


def main():
    rclpy.init()
    record_bag_node = RecordBagNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(record_bag_node, executor=executor)
    record_bag_node.destroy_node()
    rclpy.shutdown()

    return
