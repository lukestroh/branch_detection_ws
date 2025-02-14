#!/usr/bin/env python3
from __future__ import annotations
import asyncio
from concurrent.futures import Future
# from behavior_trees_python.io_tree import IOTreeNode
from behavior_trees_python.utils.events import (
    BaseEvent,
    MoveHomeEvent,
    ToggleServoEvent,
    ChangeBlackboardValueEvent,
    UpdateCSVWithEndpoint,
    UpdateCutPointEvent,

)
import numpy as np
import py_trees

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from moveit_msgs.msg import RobotState
from std_msgs.msg import Int16


class ProcessIOBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, asyncio_loop):
        super().__init__(name)

        self.blackboard = py_trees.blackboard.Blackboard()
        """
        xbox_controller = {
            "buttons": {
                0: "A",
                1: "B",
                2: "X",
                3: "Y",
                4: "LB",
                5: "RB",
                6: "view_button",
                7: "menu_button",
                8: "xbox_button",
                9: "left_joystick",
                10: "right_joystick",
                11: "share_button"
            },
            "axes": {
                0: "left_joy_x",
                1: "left_joy_y",
                2: "LT",
                3: "right_joy_x",
                4: "right_joy_y",
                5: "RT",
                6: "Dpad_x", #18,19
                7: "Dpad_y" #20,21
            }
        }
        """
        self.event_dict = {
            "move_home": MoveHomeEvent(joy_action=1),
            "toggle_servo": ToggleServoEvent(joy_action=2),
            "execute_action": ChangeBlackboardValueEvent(
                joy_action=3,
                key="execute_action",
                value=True,
                value_change=ChangeBlackboardValueEvent.ValueChange.TOGGLE,
            ),
            # "run_controllers"
            "run_teleop": ChangeBlackboardValueEvent(
                11, "teleop_control", True, ChangeBlackboardValueEvent.ValueChange.TOGGLE
            ),
            "add_point_to_csv": UpdateCSVWithEndpoint(12),
        }

        self.asyncio_loop = asyncio_loop

    def setup(self, node: Node):
        self.blackboard.set('joy_action', -999)
        self.node = node

        self.cb_group = ReentrantCallbackGroup()

        self._sub_joy_action = self.node.create_subscription(
            msg_type=Int16,
            topic="joy_action",
            callback=self._sub_cb_joy_action,
            qos_profile=10
        )

        for event in self.event_dict.values():
            event.setup(node=node, callback_group=self.cb_group, blackboard=self.blackboard)

        return
    
    def _sub_cb_joy_action(self, msg: Int16):
        self.blackboard.set('joy_action', msg.data)
        return
    
    def process_event(self, event: BaseEvent):
        """Handle events asynchronously by scheduling them on the asyncio event loop"""
        
        if event.run_async:
            self.node.get_logger().info(f"ProcessIOBehavior: hello async behavior")

            try:
                # schedule coroutine and get future
                future: Future = asyncio.run_coroutine_threadsafe(coro=event.callback(), loop=self.asyncio_loop)

                # Attach a callback to handle completion
                def on_complete(_future: Future):
                    try:
                        result = _future.result()
                        self.node.get_logger().info(f"ProcessIOBehavior: Event {event.joy_action} completed with result {result}")
                    except Exception as e:
                        self.node.get_logger().error(f"ProcessIOBehavior: Event {event.joy_action} failed: {e}")
                    return result
                
                future.add_done_callback(on_complete)
            
            except Exception as e:
                self.node.get_logger().error(f"ProcessIOBehavior: error processing Event -- {e}")
        else:
            return event.callback()
        
    def update(self):
        """Synchronously tick the behavior while scheudling async callbacks"""
        for event in self.event_dict.values():
            if self.blackboard.get('joy_action') == event.joy_action:
                self.node.get_logger().info(f"ProcessIOBehavior: Processing event {event.joy_action}")
                result = self.process_event(event=event)

        self.blackboard.set('joy_action', -999)

        return py_trees.common.Status.SUCCESS
