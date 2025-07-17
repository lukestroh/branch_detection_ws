#!/usr/bin/env python3


from rclpy.node import Node
from rcl_interfaces.srv import ListParameters

import py_trees.console as con


class LoggerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        # Loggers
        self.info = lambda x: self.get_logger().info(con.reset + con.green + f"\n{x}" + con.reset)
        self.debug = lambda x: self.get_logger().info(con.reset + con.white + f"\n{x}" + con.reset)
        self.warn = lambda x: self.get_logger().warn(con.reset + con.yellow + f"\n{x}" + con.reset)
        self.error = lambda x: self.get_logger().error(con.reset + con.red + f"\n{x}" + con.reset)
        self.fatal = lambda x: self.get_logger().fatal(con.reset + con.bold_red + f"\n{x}" + con.reset)

        # Parameters



        # # Service clients
        # self._srv_client_list_params = self.create_client(srv_name=f"{node_name}/list_parameters", srv_type=ListParameters)
        # while not self._srv_client_list_params.wait_for_service(timeout_sec=1.0):
        #     self.warn("Waiting for parameter server...")
        

        # # Timers
        # self._timer_save_node_params = self.create_timer(timer_period_sec=0.0, callback=self._timer_cb_save_node_params)


        return

    def _timer_cb_save_node_params(self) -> bool:
        

        return
