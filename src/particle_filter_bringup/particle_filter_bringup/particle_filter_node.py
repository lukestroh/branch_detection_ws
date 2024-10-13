#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration

from particle_filter_bringup.utils.particle import Particle
from particle_filter_bringup.utils.particle_filter import ParticleFilter, ParticleFilterConfig
from particle_filter_bringup.utils.tf_node import TFNode

from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

import numpy as np


class ParticleFilterNode(TFNode):
    def __init__(self) -> None:
        super().__init__(node_name="particle_filter_node")
        # Node parameters
        tof_fov_x = self.declare_parameter(name="tof_fov_x", value=Parameter.Type.DOUBLE)
        tof_fov_y = self.declare_parameter(name="tof_fov_y", value=Parameter.Type.DOUBLE)

        # Transforms
        # self.tf_frames = []
        self.tof0_frame = self.declare_parameter("tof0_frame", value="tof0").get_parameter_value().string_value # TODO: dynamically assign frames
        self.tof1_frame = self.declare_parameter("tof1_frame", value="tof1").get_parameter_value().string_value        
        
        # Publishers
        self._pub_particles = self.create_publisher(
            msg_type=Marker,
            topic="tof_particles",
            qos_profile=1
        )

        # Timers
        self._timer_pub_particles = self.create_timer(
            timer_period_sec=1/20,
            callback=self._timer_cb_pub_particles
        )
        self._timer_get_transform = self.create_timer(
            timer_period_sec=1/20,
            callback=self._timer_cb_get_transform
        )

        # Particle filter
        # We have a 25 degree FoV, can range from 0.03m - 2.0m. y and z ranges set according to max error distribution
        # self.tof_z_range = self.sensor_z_range # m
        self.tof_z_range = np.array([0.03, 2.0])
        self.tof_y_fov = np.radians([-25/2, 25/2])
        self.tof_x_fov = np.radians([-25/2, 25/2])
        self.tof_y_range = np.array([np.sin(self.tof_y_fov[0]), np.sin(self.tof_y_fov[1])]) * self.tof_z_range[1]
        self.tof_x_range = np.array([np.sin(self.tof_x_fov[0]), np.sin(self.tof_x_fov[1])]) * self.tof_z_range[1]
        

        # TODO: work ehre
        if self.tof0_world_transform is None or self.tof1_world_transform is None:
            self.pf = ParticleFilter()
            self.pf_config = ParticleFilterConfig(
                num_particles=100,
                x_range=self.tof_x_range,
                y_range=self.tof_y_range, # TODO: should this be the entire tof space, or just the overlapping tof space?
                z_range=self.tof_z_range,
            )
            self.pf.reset_filter(setup_data=self.pf_config)

        return
    
    def _timer_cb_get_transform(self) -> None:
        """Get the transforms from the tof frames to the world
            Then get the poses within the particle!
        """
        try:
            self.tof0_world_transform: np.ndarray = self.lookup_transform(target_frame="world", source_frame=self.tof0_frame, time=self.get_clock().now(), as_matrix=True)
            self.tof1_world_transform: np.ndarray = self.lookup_transform(target_frame="world", source_frame=self.tof1_frame, time=self.get_clock().now(), as_matrix=True)

            self.get_logger().warn(f"{self.tof0_world_transform}")
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup warning: {e}")
        return

    def _timer_cb_pub_particles(self) -> None:
        # """Callback timer for particles publisher"""
        
        # # self.pf.update()
        # # self.warn(self.pf.particles)

        # tf_tool0__tof0 = self.lookup_transform(target_frame=self.tof0_frame, source_frame="tool0") # TODO: source_frame needs dynamic tf_prefix value
        # tf_tool0__tof1 = self.lookup_transform(target_frame=self.tof1_frame, source_frame="tool0")
        # # self.warn(f"HELLO WORLD: {tf_tool0__tof0}")

        # msg = self._generate_particle_markers(particles=self.pf.particles)
        # self._pub_particles.publish(msg=msg)
        return
    
    def _generate_particle_markers(self, particles) -> Marker:
        """Generate markers for the particles"""
        marker = Marker()
        # marker.type = marker.POINTS
        # marker.ns = "pfilter"
        # marker.id = 0
        # marker.header.frame_id = "" # We want the output of the particle filter to be a twist... 
        #                                       # TODO: Need another marker
        #                                       # TODO: need to find center of tof transforms
        # marker.header.stamp = self.get_clock().now().to_msg()
        # marker.action = marker.ADD
        # marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in particles]
        # marker.scale.x = 0.01  # Size of the points in the x direction
        # marker.scale.y = 0.01 
        # marker.scale.z = 0.01
        # color0 = ColorRGBA(a=1.0, r=1.0, g=0.8, b=0.25)
        # marker.colors = [color0] * len(marker.points)
        return marker
    
    
    

def main():
    rclpy.init()
    particle_filter = ParticleFilterNode()
    rclpy.spin(node=particle_filter, executor=MultiThreadedExecutor())
    particle_filter.destroy_node()
    rclpy.shutdown()
    return

if __name__ == "__main__":
    main()