#!/usr/bin/env python3
# from pybullet_tree_sim import rbg_label
from xml.sax.xmlreader import XMLReader
from numpy.typing import NDArray
from pybullet_tree_sim import MESHES_PATH, URDF_PATH, RGB_LABEL
from pybullet_tree_sim.tree import Tree
from pybullet_tree_sim.utils.pyb_utils import PyBUtils

from collections import defaultdict
import cv2
import glob
import gymnasium as gym
import numpy as np
import os
import skimage.draw as skdraw
import sys
from typing import Optional, Tuple
import xml


class PruningEnvException(Exception):
    def __init__(self, s, *args):
        super().__init__(args)
        self.s = s
        return

    def __str__(self):
        return f"{self.s}"


class PruningEnv(gym.Env):
    rgb_label = RGB_LABEL
    """
        PruningEnv is a custom environment that extends the gym.Env class from OpenAI Gym.
        This environment simulates a pruning task where a robot arm interacts with a tree.
        The robot arm is a UR5 arm and the tree is a 3D model of a tree.
        The environment is used to train a reinforcement learning agent to prune the tree.
    """

    def __init__(
        self,
        # angle_threshold_perp: float = 0.52,
        # angle_threshold_point: float = 0.52,
        cam_width: int = 424,
        cam_height: int = 240,
        evaluate: bool = False,
        # distance_threshold: float = 0.05,
        max_steps: int = 1000,
        make_trees: bool = False,
        name: str = "PruningEnv",
        num_trees: int | None = None,
        pbutils: PyBUtils | None = None,
        renders: bool = False,
        tree_count: int = 10,
        tree_urdf_path: str | None = None,
        tree_obj_path: str | None = None,
        verbose: bool = True
        # use_ik: bool = True,
    ) -> None:
        """Initialize the Pruning Environment

        Parameters
        ----------
        cam_width (int): Pixel width of the camera
        cam_height (int): Pixel height of the camera
        evaluate (bool): Is this environment for evaluation
        max_steps (int): Maximum number of steps in a single run
        name (str): Name of the environment (default="PruningEnv")
        renders (bool): Whether to render the environment
        tree_count (int): Number of trees to be loaded
        """
        super().__init__()
        self.pbutils: PyBUtils
        if pbutils is not None:
            self.pbutils = pbutils

        # Pybullet GUI variables
        self.render_mode = "rgb_array"
        self.renders = renders
        self.eval = evaluate

        # # Obj/URDF paths
        # self.tree_urdf_path = tree_urdf_path
        # self.tree_obj_path = tree_obj_path
        # # self.tree_labelled_path = tree_labelled_path
        # self.tree_id = None

        # Gym variables
        self.name = name
        self.step_counter = 0
        self.global_step_counter = 0
        # self.max_steps = max_steps
        self.tree_count = tree_count
        self.is_goal_state = False

        # Camera params
        self.cam_width = cam_width
        self.cam_height = cam_height
        self.cam_pan = 0
        self.cam_tilt = 0
        self.cam_xyz_offset = np.zeros(3)

        self.verbose = verbose

        self.collision_object_ids = {
            "SPUR": None,
            "TRUNK": None,
            "BRANCH": None,
            "WATER_BRANCH": None,
            "SUPPORT": None,
        }

        # Tree parameters
        # self.tree_goal_pos = None
        # self.tree_goal_or = None
        # self.tree_goal_normal = None
        self.tree_urdf_path: str | None = None
        self.tree_pos = np.zeros(3, dtype=float)
        self.tree_orientation = np.zeros(3, dtype=float)
        self.tree_scale: float = 1.0

        # Load trees
        self.trees = np.empty(shape=tree_count, dtype=Tree)
        # if tree_urdf_path is None:
        #     ...

        return

    def _load_pbutils(pbutils: PyBUtils) -> None:
        self.pbutils = pbutils
        return

    def load_tree(
        self,
        scale: float,
        tree_id: int = None,
        tree_type: str = None,
        tree_namespace: str = "",
        tree_urdf_path: str | None = None,
        save_tree_urdf: bool = False,
        randomize_pose: bool = False
    ) -> None:
        if randomize_pose:
            ...

        # If user supplied URDF file, check if it exists
        if tree_urdf_path is not None:
            # generated_trees = glob.glob(os.path.join(URDF_PATH, "trees", tree_type, "generated/*.urdf"))
            # matches = list(map(lambda x: x.endswith(f"{tree_namespace}{tree_type}_tree{tree_id}.urdf"), generated_trees))
            path_exists = os.path.exists(tree_urdf_path)
            # if not any(matches):
            if not path_exists:
                raise OSError(
                    f"There do not seem to be any files of that name, please check your path. Given path was {tree_urdf_path}"
                )

        # Get tree object
        tree = Tree.create_tree(
            tree_id=tree_id, tree_type=tree_type, scale=scale, namespace=tree_namespace, tree_urdf_path=tree_urdf_path
        )

        # # Get URDF from tree
        # tree_urdf = Tree.get_tree_urdf(
        #     tree_id=tree_id, tree_type=tree_type, scale=scale, namespace=tree_namespace, tree_urdf_path=tree_urdf_path
        # )
        # # tree_urdf = tree.urdf


        urdf_path = os.path.join(
            URDF_PATH, "trees", tree_type, "generated", f"{tree_namespace}{tree_type}_tree{tree_id}.urdf"
        )

        # Save tree URDF
        if save_tree_urdf:
            with open(os.path.join(urdf_path), "w") as file:
                file.write(tree_urdf)
            print(f"File {urdf_path} saved")

        # self.pbutils.pbclient.loadURDF(urdf_path)

        return


def main():
    from pybullet_tree_sim.tree import Tree

    penv = PruningEnv()
    penv.pbutils = PyBUtils(renders=False, cam_height=64, cam_width=64)
    penv.load_tree(
        scale=1.0,
        tree_id=0,
        tree_type="envy",
        tree_namespace="LPy_",
        # tree_urdf_path=os.path.join(URDF_PATH, "trees", "envy", "generated", "LPy_envy_tree0.urdf"),
        save_tree_urdf=True,
    )

    return


if __name__ == "__main__":
    main()
