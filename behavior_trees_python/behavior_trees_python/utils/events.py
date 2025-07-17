#!/usr/bin/env python3
from __future__ import annotations
from abc import ABC, abstractmethod
from enum import Enum
import numpy as np
import py_trees
from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup
from rclpy.task import Future
import re

# from behavior_trees_python.io_tree import IOTreeNode

from branch_detection_system_moveit_msgs.srv import MoveToJointAngles
from controller_manager_msgs.srv import SwitchController, ListControllers

from std_srvs.srv import Trigger


class BaseEvent(ABC):
    def __init__(self, joy_action):
        self.joy_action = joy_action
        self.run_async = None
        self.node = None
        self.cb_group = None
        return

    @abstractmethod
    def setup(self, node, callback_group, blackboard):
        pass


class AsyncEvent(BaseEvent):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        self.run_async = True

    @abstractmethod
    async def callback(self):
        pass


class BlockingEvent(BaseEvent):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        self.run_async = False

    @abstractmethod
    def callback(self):
        pass


class ChangeBlackboardValueEvent(BlockingEvent):
    class ValueChange(Enum):
        SET = 0
        TOGGLE = 1

    def __init__(self, joy_action, key, value, value_change, default_value=False):
        super().__init__(joy_action)
        self.key = key
        self.value = value
        self.value_change = value_change
        self.default_value = default_value
        return

    def setup(self, node: IOTreeNode, callback_group: CallbackGroup, blackboard: py_trees.blackboard.Blackboard):
        self.blackboard = blackboard
        self.node = node
        self.cb_group = callback_group
        self.blackboard.set(self.key, self.value)
        return

    def change_blackboard_value(self):
        if self.value_change == self.ValueChange.TOGGLE:
            self.value = not self.blackboard.get(self.key)

        self.blackboard.set(self.key, self.value)
        self.node.get_logger().info(f"Setting blackboard value {self.key} to {self.value}")
        return True

    def callback(self):
        self.change_blackboard_value()
        return


class MoveHomeEvent(AsyncEvent):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        self.home_joint_angles = [-np.pi / 2, -np.pi * 2 / 3, np.pi * 2 / 3, -np.pi, -np.pi / 2, 0.0]
        return

    def setup(self, node: Node, callback_group: CallbackGroup, blackboard: py_trees.blackboard.Blackboard):
        self.blackboard = blackboard
        self.node = node
        self.cb_group = callback_group
        self._srv_client_move_to_joint_state = node.create_client(
            srv_type=MoveToJointAngles, srv_name="/move_to_joint_angles", callback_group=self.cb_group
        )
        return

    async def move_to_joint_angles(self, joint_angles):
        self.arm_prefix = "ur5e__"
        request = MoveToJointAngles.Request()
        request.joint_angles = joint_angles
        request.joint_names = [
            f"{self.arm_prefix}shoulder_pan_joint",  # TODO: Dynamically get list of robot joint names ()
            f"{self.arm_prefix}shoulder_lift_joint",
            f"{self.arm_prefix}elbow_joint",
            f"{self.arm_prefix}wrist_1_joint",
            f"{self.arm_prefix}wrist_2_joint",
            f"{self.arm_prefix}wrist_3_joint",
        ]

        move_request_future: Future = self._srv_client_move_to_joint_state.call_async(request=request)
        await move_request_future

        return move_request_future.result()

    async def move_home_callback(self):
        self.node.get_logger().info("Moving to home position")

        move_result = await self.move_to_joint_angles(self.home_joint_angles)

        return move_result

    async def callback(self):
        move_result = await self.move_home_callback()
        return move_result


class ToggleServoEvent(AsyncEvent):
    def __init__(self, joy_action):
        super().__init__(joy_action)

        self.planning_ctrlr_name = None
        self.servo_ctrlr_name = None
        return

    class ControllerMode(Enum):
        PLANNING = 0
        SERVO = 1

    def setup(self, node: Node, callback_group, blackboard: py_trees.blackboard.Blackboard):
        self.node = node
        self.cb_group = callback_group
        self.blackboard = blackboard

        # Params
        self._param_planning_ctrlr = self.node.declare_parameter("planning_controller", ".*joint_trajectory_controller")
        self._param_servo_ctrlr = self.node.declare_parameter("servo_controller", "forward_position_controller")
        # node.warn(f'{self._param_servo_ctrlr}')

        # Service clients
        self._srv_client_start_servo = self.node.create_client(
            srv_type=Trigger, srv_name="/servo_node/start_node", callback_group=self.cb_group
        )
        self._srv_client_stop_servo = self.node.create_client(
            srv_type=Trigger, srv_name="/servo_node/stop_node", callback_group=self.cb_group
        )
        self._srv_client_switch_controllers = self.node.create_client(
            srv_type=SwitchController, srv_name="/controller_manager/switch_controllers", callback_group=self.cb_group
        )
        self._srv_client_list_controllers = self.node.create_client(
            srv_type=ListControllers, srv_name="/controller_manager/list_controllers", callback_group=self.cb_group
        )

        # Timers
        self._timer_get_ctrlr_names = self.node.create_timer(
            timer_period_sec=0.1, callback=self._timer_cb_get_ctrlr_names, callback_group=self.cb_group
        )

        # Attributes
        self.controller_mode = self.ControllerMode.SERVO
        self.blackboard.set("controller_mode", self.controller_mode)
        return

    async def _timer_cb_get_ctrlr_names(self):
        if self.planning_ctrlr_name is not None:
            self._timer_get_ctrlr_names.destroy()

        if not self._srv_client_list_controllers.service_is_ready():
            return

        list_ctrls_resp: Future = self._srv_client_list_controllers.call_async(ListControllers.Request())
        await list_ctrls_resp
        list_ctrls_result: ListControllers.Response = list_ctrls_resp.result()

        for controller in list_ctrls_result.controller:
            if self.planning_ctrlr_name is None and re.match(self._param_planning_ctrlr.value, controller.name):
                self.planning_ctrlr_name = controller.name

            if self.servo_ctrlr_name is None and re.match(self._param_servo_ctrlr.value, controller.name):
                self.servo_ctrlr_name = controller.name

        if bool(self.planning_ctrlr_name) ^ bool(self.servo_ctrlr_name):
            print("Only was able to match one of the controllers! Not activating")
            self.planning_ctrlr_name = None
            self.servo_ctrlr_name = None

        elif self.planning_ctrlr_name is not None:
            print(f"Located controllers! Base: {self.planning_ctrlr_name}, Servo: {self.servo_ctrlr_name}")

    async def handle_resource_switch(self):
        if self.planning_ctrlr_name is None or self.servo_ctrlr_name is None:
            raise Exception("Controllers have not yet been identified!")

        if self.controller_mode != self.ControllerMode.PLANNING:
            switch_ctrlr_req = SwitchController.Request(
                activate_controllers=[self.planning_ctrlr_name],
                deactivate_controllers=[self.servo_ctrlr_name],
                strictness=SwitchController.Request.BEST_EFFORT,
            )
            stop_servo_future: Future = self._srv_client_stop_servo.call_async(Trigger.Request())
            await stop_servo_future
            switch_ctrlr_future: Future = self._srv_client_switch_controllers.call_async(switch_ctrlr_req)
            await switch_ctrlr_future
            if not switch_ctrlr_future.result().ok:
                raise Exception("Failed to switch controllers!")
            else:
                self.controller_mode = self.ControllerMode.PLANNING

        elif self.controller_mode != self.ControllerMode.SERVO:
            switch_ctrlr_req = SwitchController.Request(
                activate_controllers=[self.servo_ctrlr_name],
                deactivate_controllers=[self.planning_ctrlr_name],
                strictness=SwitchController.Request.BEST_EFFORT,
            )

            switch_ctrlr_future: Future = self._srv_client_switch_controllers.call_async(switch_ctrlr_req)
            await switch_ctrlr_future
            if not switch_ctrlr_future.result().ok:
                raise Exception("Failed to switch controllers!")
            else:
                start_servo_future: Future = self._srv_client_start_servo.call_async(Trigger.Request())
                await start_servo_future
                self.controller_mode = self.ControllerMode.SERVO

        else:
            raise ValueError(f"Unknown controller mode specified: {self.controller_mode}")

        self.node.get_logger().info(f"Controller mode switched to {self.controller_mode}")

        self.blackboard.set("controller_mode", self.controller_mode)

        return

    async def callback(self):
        self.node.get_logger().info("")
        await self.handle_resource_switch()
        return True


class UpdateCutPointEvent(ChangeBlackboardValueEvent):
    csv_index = -1

    def __init__(self, joy_action, key, value, value_change, default_value=False):
        super().__init__(joy_action, key, value, value_change, default_value)
        self.csv_file = "goal_log.csv"
        return

    def setup(self, node, callback_group, blackboard):
        super().setup(node, callback_group, blackboard)

    def callback(self):
        # Read the csv file
        if self.joy_action < 0:
            UpdateCutPointEvent.csv_index += 1
        elif self.joy_action > 0:
            UpdateCutPointEvent.csv_index -= 1
        with open(self.csv_file, "r") as f:
            lines = f.readlines()
            if self.csv_index < len(lines):
                line = lines[self.csv_index]
                values = line.split(",")
                x = float(values[0])
                y = float(values[1])
                z = float(values[2])
                self.value = (x, y, z)
                self.node.get_logger().info(
                    f"Setting blackboard value {self.key} to {self.value} with index {self.csv_index}"
                )
                return self.change_blackboard_value()
            else:
                UpdateCutPointEvent.csv_index = len(lines) - 1
                self.node.get_logger().info("End of csv file reached")
                return False


class UpdateCSVWithEndpoint(AsyncEvent):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        return

    def setup(self, node: Node, callback_group, blackboard: py_trees.blackboard.Blackboard):
        self.node = node
        self.cb_group = callback_group
        self._srv_client_update_csv = self.node.create_client(srv_type=Trigger, srv_name="/set_point_from_endpoint")
        self._srv_client_update_csv.wait_for_service()
        return

    async def callback(self):
        self.node.warn("UpdateCSVWithEndpointEvent: WE GOT HERE")

        self._srv_client_update_csv.call_async(Trigger.Request())
        return
