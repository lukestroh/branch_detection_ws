#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future

from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

from final_approach_controller.tf_node import TFNode

from process_io_msgs.srv import SetPoint

import traceback


class SetPointFromEndpointServiceNode(TFNode):
    def __init__(self):
        super().__init__(node_name="set_point_from_endpoint_service_node")
        # Loggers
        self.info = lambda x: self.get_logger().info(f"\n{x}")
        self.warn = lambda x: self.get_logger().warn(f"\n{x}")
        self.err = lambda x: self.get_logger().error(f"\n{x}")

        # Parameters
        self._param_robot_base_part = self.declare_parameter("robot_base_part", value=Parameter.Type.STRING)
        self._param_robot_eef_part = self.declare_parameter("robot_eef_part", value=Parameter.Type.STRING)
        self.warn(self._param_robot_base_part.get_parameter_value().string_value)

        # Callback groups
        self.cb_group = ReentrantCallbackGroup()

        # Service servers
        self._srv_server_set_point_from_endpoint = self.create_service(
            srv_name="set_point_from_endpoint",
            srv_type=Trigger,
            callback=self._srv_cb_set_point_from_endpoint,
            callback_group=self.cb_group,
        )

        # Service clients
        self._srv_client_set_point = self.create_client(
            srv_name="set_point", srv_type=SetPoint, callback_group=self.cb_group
        )
        while not self._srv_client_set_point.wait_for_service(timeout_sec=1.0):
            self.warn("Service 'set_point' not available. Trying again...")

        return

    async def _srv_cb_set_point_from_endpoint(self, request: Trigger.Request, response: Trigger.Response):
        """When triggered, get transform from base link to eef tool0 and call set_point service."""
        try:
            tf = self.lookup_transform(
                target_frame=f"{self._param_robot_base_part}base",
                source_frame=f"{self._param_robot_eef_part}tool0",
                time=self.get_clock().now(),
                sync=True,
            )

            new_goal = Point()
            new_goal.x = tf.transform.translation.x
            new_goal.y = tf.transform.translation.y
            new_goal.z = tf.transform.translation.z

            set_point_req = SetPoint.Request()
            set_point_req.position = new_goal

            self.info(f"Setting point from eef tool0 frame: ({new_goal.x}, {new_goal.y}, {new_goal.z})")

            set_point_future: Future = self._srv_client_set_point.call_async(set_point_req)
            await set_point_future
            set_point_res: SetPoint.Result = set_point_future.result()

            if set_point_res is not None or set_point_res.success:
                response.success = True
                response.message = "Goal successfully set from eef tool0"
            else:
                response.success = False
                response.message = "Error calling set_point service"

        except Exception as e:
            self.err(traceback.format_exc())
            response.success = False
            response.message = f"Error: {e}"

        return response


def main():
    rclpy.init()
    save_point_from_endpoint_service_node = SetPointFromEndpointServiceNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(save_point_from_endpoint_service_node, executor=executor)
    save_point_from_endpoint_service_node.destroy_node()
    rclpy.shutdown()

    return
