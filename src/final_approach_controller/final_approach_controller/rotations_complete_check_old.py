 if not self.rotations_complete:
    if not self.neg_rot_complete:
        # # rotate to the closest side
        # if self.joint_states[-1] < 0 and self.joint_states[-1] > -1 * np.pi:
        # negative angular rotation
        angular_z = -1 * self.max_angular_vel
        if np.isclose(
            self.joint_states[2], np.pi / 2, atol=0.01
        ):  # TODO: (long term) make sure wrist mount config is standard
            # Try to fit data
            self.tof0_time_center, self.tof0_distance_center = (
                self.get_branch_center_time_and_distance(
                    timestamps=self.timestamp_readings,
                    readings=self.d_tof0_readings,
                    sensor_name="tof0",
                    debug_plot=self.debug_plot,
                )
            )
            self.tof1_time_center, self.tof1_distance_center = (
                self.get_branch_center_time_and_distance(
                    timestamps=self.timestamp_readings,
                    readings=self.d_tof1_readings,
                    sensor_name="tof1",
                    debug_plot=self.debug_plot,
                )
            )
            if self.tof0_time_center is not None and self.tof1_time_center is not None:
                self.publish_zero_twist()
                self.rotations_complete = True

            self.neg_rot_complete = True

    elif not self.pos_rot_complete:
        # if self.joint_states[-1] > 0 and self.joint_states[-1] < np.pi:
        # positive angular rotation
        angular_z = self.max_angular_vel
        if np.isclose(self.joint_states[2], np.pi + np.pi / 2, atol=0.01):
            # Try to fit data TODO: if fit is upside down, nix it
            self.tof0_time_center, self.tof0_distance_center = (
                self.get_branch_center_time_and_distance(
                    timestamps=self.timestamp_readings,
                    readings=self.d_tof0_readings,
                    sensor_name="tof0",
                    debug_plot=self.debug_plot,
                )
            )

            self.tof1_time_center, self.tof1_distance_center = (
                self.get_branch_center_time_and_distance(
                    timestamps=self.timestamp_readings,
                    readings=self.d_tof1_readings,
                    sensor_name="tof1",
                    debug_plot=self.debug_plot,
                )
            )
            if self.tof0_time_center is not None and self.tof1_time_center is not None:
                self.publish_zero_twist()
                self.rotations_complete = True

            self.pos_rot_complete = True

    self.msg_twist.twist.linear.x = 0.0
    self.msg_twist.twist.linear.y = 0.0
    self.msg_twist.twist.linear.z = 0.0
    self.msg_twist.twist.angular.x = 0.0
    self.msg_twist.twist.angular.y = 0.0
    self.msg_twist.twist.angular.z = angular_z
    self.msg_twist.header.frame_id = "mock_pruner__tool0"  # TODO: Get name dynamically
    self.msg_twist.header.stamp = self.get_clock().now().to_msg()
    self._pub_servo.publish(self.msg_twist)

    if self.neg_rot_complete and self.pos_rot_complete:
        self.publish_zero_twist()
        self.rotations_complete = True