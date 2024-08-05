#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def launch_setup(context, *args, **kwargs):

    node_particle_filter = Node(
        package="particle_filter_bringup",
        executable="particle_filter_node",
        name="particle_filter"
    )

    nodes_to_start = [node_particle_filter]

    return nodes_to_start

def generate_launch_description():
    declared_args = []

    ld = LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])

    return ld