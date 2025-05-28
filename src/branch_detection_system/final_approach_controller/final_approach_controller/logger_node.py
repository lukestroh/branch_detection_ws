#!/usr/bin/env python3


from rclpy.node import Node

import py_trees.console as con


class LoggerNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.info = lambda x: self.get_logger().info(con.reset + con.white + f"\n{x}" + con.reset)
        self.debug = lambda x: self.get_logger().info(con.reset + con.green + f"\n{x}" + con.reset)
        self.warn = lambda x: self.get_logger().warn(con.reset + con.yellow + f"\n{x}" + con.reset)
        self.error = lambda x: self.get_logger().error(con.reset + con.bold_red + f"\n{x}" + con.reset)
        self.fatal = lambda x: self.get_logger().fatal(con.reset + con.bold_red + f"\n{x}" + con.reset)

        return
