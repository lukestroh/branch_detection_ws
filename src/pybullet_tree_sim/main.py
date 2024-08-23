#!/usr/bin/env python3

from pybullet_tree_sim.pyb_environment import PyBEnvironment
from pybullet_tree_sim.tree import Tree
from pybullet_tree_sim.utils import pybullet_utils, ur5_utils

def main():
    # env: The environment object
    # pyb: The pybullet object
    # urdf_path: The path to the urdf file
    # obj_path: The path to the obj file
    # labelled_obj_path: The path to the labelled obj file

    
    env = PyBEnvironment()
    
    # From Tree originally
    # self.base_xyz = env.ur5.get_current_pose(env.ur5.base_index)[0]
    # self.ee_xyz = env.ur5.get_current_pose(env.ur5.end_effector_index)[0]

    return

if __name__ == "__main__":
    main()
