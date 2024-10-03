#!/usr/bin/env python3
from xml.sax.xmlreader import XMLReader
from numpy.typing import NDArray
from plotly.graph_objs import Violin
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from pybullet_tree_sim import MESHES_PATH, URDF_PATH, RGB_LABEL, ROBOT_URDF_PATH
from pybullet_tree_sim.tree import Tree, TreeException
from pybullet_tree_sim.utils.ur5_utils import UR5
from pybullet_tree_sim.utils.pyb_utils import PyBUtils
import pybullet_tree_sim.utils.xacro_utils as xutils


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

import modern_robotics as mr
from numpy.typing import ArrayLike

from zenlog import log


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

    _supports_and_post_xacro_path = os.path.join(URDF_PATH, "supports_and_post", "supports_and_post.urdf.xacro")
    _supports_and_post_urdf_path = os.path.join(URDF_PATH, "supports_and_post", "supports_and_post.urdf")

    def __init__(
        self,
        # angle_threshold_perp: float = 0.52,
        # angle_threshold_point: float = 0.52,
        pbutils: PyBUtils,
        cam_width: int = 424,
        cam_height: int = 240,
        evaluate: bool = False,
        # distance_threshold: float = 0.05,
        max_steps: int = 1000,
        make_trees: bool = False,
        name: str = "PruningEnv",
        num_trees: int | None = None,
        renders: bool = False,
        tree_count: int = 10,
        # tree_urdf_path: str | None = None,
        # tree_obj_path: str | None = None,
        verbose: bool = True,
        load_robot: bool = True,
        # robot_type: str = "ur5e",
        robot_pos: ArrayLike = np.array([0, 0, 0]),
        robot_orientation: ArrayLike = np.array([0, 0, 0, 1]),
        # use_ik: bool = True,
        #
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
        self.pbutils = pbutils

        # Pybullet GUI variables
        self.render_mode = "rgb_array"
        self.renders = renders
        self.eval = evaluate

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
        # self.cam_xyz_offset = np.zeros(3)
        self.cam_xyz_offset = np.array([0, 0, 0])
        # Normalize pixel coordinates

        # Camera stuff
        self.pixel_coords = np.array(list(np.ndindex((cam_height, cam_width))), dtype=int)
        # Find the pixel coordinates in the film plane. Bin, normalize, and offset the pixel coordinates
        self.film_plane_coords = np.zeros((cam_height, cam_width, 2), dtype=float)
        self.film_plane_coords = np.subtract(np.divide(np.add(self.pixel_coords, [0.5, 0.5]), [cam_height, cam_width]), 0.5) # TODO: define this last subtraction as an offset?

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
        # self.tree_urdf_path: str | None = None
        # self.tree_pos = np.zeros(3, dtype=float)
        # self.tree_orientation = np.zeros(3, dtype=float)
        # self.tree_scale: float = 1.0

        # Trees
        self.trees: dict[str, Tree] = {}

        # UR5 Robot
        if load_robot:
            self.ur5 = self.load_robot(robot_pos=robot_pos, robot_orientation=robot_orientation)
        return

    def load_robot(self, robot_pos: ArrayLike, robot_orientation: ArrayLike, randomize_pose: bool = False):
        log.info("Loading UR5 Robot")
        robot = UR5(
            con=self.pbutils.pbclient,
            robot_urdf_path=ROBOT_URDF_PATH,
            pos=robot_pos,
            orientation=robot_orientation,
            randomize_pose=randomize_pose,
            verbose=self.verbose,
        )
        return robot

    def load_tree(
        self,
        pbutils: PyBUtils,
        scale: float,
        position: ArrayLike = [0, 0, 0],
        orientation: ArrayLike = [0, 0, 0, 1],
        tree_id: int = None,
        tree_type: str = None,
        tree_namespace: str = "",
        tree_urdf_path: str | None = None,
        save_tree_urdf: bool = False,
        randomize_pose: bool = False,
    ) -> None:
        if randomize_pose:
            ...

        # If user supplied URDF file, check if it exists
        if tree_urdf_path is not None:
            if not os.path.exists(tree_urdf_path):
                raise OSError(
                    f"There do not seem to be any files of that name, please check your path. Given path was {tree_urdf_path}"
                )

        # Get tree object
        tree = Tree.create_tree(
            pbutils=pbutils,
            scale=scale,
            position=position,
            orientation=orientation,
            tree_id=tree_id,
            tree_type=tree_type,
            namespace=tree_namespace,
            tree_urdf_path=tree_urdf_path,
            randomize_pose=randomize_pose,
        )

        # tree_id_str = f"{tree_namespace}{tree_type}_tree{tree_id}"
        urdf_path = os.path.join(URDF_PATH, "trees", tree_type, "generated", f"{tree.id_str}.urdf")

        # Add tree to dict of trees
        self.trees[tree.id_str] = tree
        return

    def activate_tree(
        self, tree: Tree | None = None, tree_id_str: str | None = None, include_support_posts: bool = True
    ) -> None:
        if tree is None and tree_id_str is None:
            raise TreeException("Parameters 'tree' and 'tree_id_str' cannot both be None")

        if tree is None and tree_id_str is not None:
            try:
                tree = self.trees[tree_id_str]
            except KeyError as e:
                raise TreeException(f"{e}: Tree with ID {tree_id_str} not found")

        if tree is not None:
            if self.verbose:
                log.info("Activating tree")
            tree.pyb_tree_id = self.pbutils.pbclient.loadURDF(tree.urdf_path, useFixedBase=True)
            log.info(f"Tree {tree.id_str} activated with PyBID {tree.pyb_tree_id}")

        # Activate support posts around the tree
        if include_support_posts:
            self.activate_support_posts(associated_tree=tree)
        return

    def deactivate_tree(self, tree: Tree | None = None, tree_id_str: str | None = None) -> None:
        if tree is None and tree_id_str is None:
            raise TreeException("Parameters 'tree' and 'tree_id_str' cannot both be None")

        if tree is None and tree_id_str is not None:
            try:
                tree = self.trees[tree_id_str]
            except KeyError as e:
                raise TreeException(f"{e}: Tree with ID {tree_id_str} not found")

        if tree is not None:
            try:
                self.pbutils.pbclient.removeBody(tree.pyb_tree_id)
                log.info(f"Tree {tree.id_str} with PyBID {tree.pyb_tree_id} deactivated")
            except Exception as e:
                log.error(f"Error deactivating tree: {e}")
        return

    def activate_support_posts(
        self, associated_tree: Tree, position: ArrayLike | None = None, orientation: ArrayLike | None = None
    ) -> None:
        # Load support posts
        if position is None:
            position = [associated_tree.pos[0], associated_tree.pos[1] - 0.05, 0.0]
        if orientation is None:
            orientation = Rotation.from_euler("xyz", [np.pi / 2, 0, np.pi / 2]).as_quat()

        if not os.path.exists(self._supports_and_post_urdf_path):
            urdf_content = xutils.load_urdf_from_xacro(
                xacro_path=self._supports_and_post_xacro_path, mappings=None
            ).toprettyxml()  # TODO: add mappings
            xutils.save_urdf(urdf_content=urdf_content, urdf_path=self._supports_and_post_urdf_path)

        support_post = self.pbutils.pbclient.loadURDF(
            fileName=self._supports_and_post_urdf_path, basePosition=position, baseOrientation=orientation
        )
        log.info(f"Supports and post activated with PyBID {support_post}")
        self.collision_object_ids["SUPPORT"] = support_post
        return

    def deproject_pixels_to_points(self, data: np.ndarray):
        """Compute world XYZ from image XY and measured depth.
        (pixel_coords -- [u,v]) -> (film_coords -- [x,y]) -> (camera_coords -- [X, Y, Z]) -> (world_coords -- [U, V, W])

        https://ksimek.github.io/2013/08/13/intrinsic/
        https://gachiemchiep.github.io/cheatsheet/camera_calibration/

        @param data: 2D array of depth values

        TODO: change all `reshape` to `resize`
        """
        # Get view and projection matrices
        view_matrix = np.asarray(
            self.ur5.get_view_mat_at_curr_pose(pan=self.cam_pan, tilt=self.cam_tilt, xyz_offset=self.cam_xyz_offset)
        ).reshape([4, 4], order="F")
        # log.debug(view_matrix)
        # log.warning(self.pbutils.proj_mat)
        proj_matrix = np.asarray(self.pbutils.proj_mat).reshape([4, 4], order="F")
        # log.debug(proj_matrix)
        # Get camera intrinsics from projection matrix
        # fx = proj_matrix[0, 0]
        # fy = proj_matrix[1, 1] # if square camera, these should be the same
        # if not np.isclose(fx, fy, atol=1e-3):
        #     log.warning("Camera is not square")


        # cam_coords = np.divide(np.multiply(self.film_plane_coords, data), [fx, fy])
        cam_coords = np.concatenate((self.film_plane_coords, data, np.ones((64,1))), axis=1)
        # print(cam_coords)
        # cam_coords = (np.array([[1/fx, 0,0],[0, 1/fy, 0],[0,0,1],[0,0,0]]) @ cam_coords.T).T
        cam_coords = (np.linalg.inv(proj_matrix) @ cam_coords.T).T
        cam_coords = cam_coords / cam_coords[:, 3].reshape(-1, 1)

        log.debug(cam_coords)



        res = (mr.TransInv(view_matrix) @ cam_coords.T).T
        # res = np.divide(res, res.reshape(-1, 1, order="F"))
        # log.debug(res)

        _data = data.reshape([8, 8], order="F")
        _data = _data.reshape((64,1), order="C")
        log.debug(_data)
        # # log.debug(reshaped_cam_coords)
        import plotly.graph_objects as go
        fig = go.Figure(
            data=[go.Scatter3d(
                x=list(range(8)) * 8,
                y=np.array([list(range(8))] * 8).T.flatten(order='C'),
                z=_data.flatten(order='C'),
                mode='markers'
            )]
        )
        fig.update_layout(
            title="Pixel Coordinates"
        )
        fig.show()
        fig = go.Figure(data=[go.Scatter3d(x=cam_coords[:,0], y=cam_coords[:,1], z=cam_coords[:,2], mode='markers')])
        fig.update_layout(
            # scene=dict(
            #     aspectmode='cube',
            #     xaxis=dict(range=[-0.1, 0.1]),
            #     yaxis=dict(range=[-0.1, 0.1]),
            #     zaxis=dict(range=[-0.0, 0.4]),
            # ),
            title="Camera Coordinates",
        )
        fig.show()
        fig = go.Figure(data=[go.Scatter3d(x=res[:,0], y=res[:,1], z=res[:,2], mode='markers')])
        fig.update_layout(
            title="World Coordinates"
        )
        fig.show()

        return




    def compute_deprojected_point_mask(self):
        # TODO: Make this function nicer
        # Function. Be Nice.
        # """Find projection stuff in 'treefitting'. Simpole depth to point mask conversion."""
        # point_mask = np.zeros((self.pbutils.cam_height, self.pbutils.cam_width), dtype=np.float32)

        # proj_matrix = np.asarray(self.pbutils.proj_mat).reshape([4, 4], order="F")
        # view_matrix = np.asarray(
            # self.ur5.get_view_mat_at_curr_pose(pan=self.cam_pan, tilt=self.cam_tilt, xyz_offset=self.cam_xyz_offset)
        # ).reshape([4, 4], order="F")
        # projection = (
        #     proj_matrix
        #     @ view_matrix
        #     @ np.array([0.5, 0.5, 1, 1])
        #     # @ np.array([self.tree_goal_pos[0], self.tree_goal_pos[1], self.tree_goal_pos[2], 1])
        # )

        # # Normalize by w -> homogeneous coordinates
        # projection = projection / projection[3]
        # # log.info(f"View matrix: {view_matrix}")
        # log.info(f"Projection: {projection}")
        # # if projection within 1,-1, set point mask to 1
        # if projection[0] < 1 and projection[0] > -1 and projection[1] < 1 and projection[1] > -1:
        #     projection = (projection + 1) / 2
        #     row = self.pbutils.cam_height - 1 - int(projection[1] * (self.pbutils.cam_height))
        #     col = int(projection[0] * self.pbutils.cam_width)
        #     radius = 5  # TODO: Make this a variable proportional to distance
        #     # modern scikit uses a tuple for center
        #     rr, cc = skdraw.disk((row, col), radius)
        #     print(rr, cc)
        #     point_mask[
        #         np.clip(0, rr, self.pbutils.cam_height - 1), np.clip(0, cc, self.pbutils.cam_width - 1)
        #     ] = 1  # TODO: This is a hack, numbers shouldnt exceed max and min anyways

        # # resize point mask to algo_height, algo_width
        # point_mask_resize = cv2.resize(point_mask, dsize=(self.algo_width, self.algo_height))
        # point_mask = np.expand_dims(point_mask_resize, axis=0).astype(np.float32)
        return point_mask

    def is_reachable(self, vertex: Tuple[np.ndarray], base_xyz: np.ndarray) -> bool:
        # if vertex[3] != "SPUR":
        #     return False
        ur5_base_pos = np.array(base_xyz)

        # Meta condition
        dist = np.linalg.norm(ur5_base_pos - vertex[0], axis=-1)

        if dist >= 0.98:  # TODO: is this for the UR5? Should it be from a parameter file?
            return False

        j_angles = self.ur5.calculate_ik(vertex[0], None)
        # env.ur5.set_joint_angles(j_angles)
        # for _ in range(100):
        #     pyb.con.stepSimulation()
        # ee_pos, _ = env.ur5.get_current_pose(env.ur5.end_effector_index)
        # dist = np.linalg.norm(np.array(ee_pos) - vertex[0], axis=-1)
        # if dist <= 0.03:
        #     return True

        return False

    def get_reachable_points(self, tree: Tree, env, pyb):
        # self.reachable_points = list(filter(lambda x: self.is_reachable(x, env, pyb), self.vertex_and_projection))
        # np.random.shuffle(self.reachable_points)
        # print("Number of reachable points: ", len(self.reachable_points))
        # if len(self.reachable_points) < 1:
        #     print("No points in reachable points", self.urdf_path)
        #     # self.reset_tree()

        return self.reachable_points


def main():
    import time
    from pybullet_tree_sim.tree import Tree
    import secrets

    cam_dfov = 65
    cam_height = 8
    cam_width = 8

    pbutils = PyBUtils(renders=False, cam_height=cam_height, cam_width=cam_width, dfov=cam_dfov)
    penv = PruningEnv(pbutils=pbutils, load_robot=True, robot_pos=[0, 1, 0], verbose=True, cam_width=cam_width, cam_height=cam_height)
    penv.load_tree(
        pbutils=pbutils,
        scale=1.0,
        tree_id=1,
        tree_type="envy",
        tree_namespace="LPy_",
        # tree_urdf_path=os.path.join(URDF_PATH, "trees", "envy", "generated", "LPy_envy_tree0.urdf"),
        save_tree_urdf=False,
        # randomize_pose=True
    )
    # penv.activate_tree(tree_id_str="LPy_envy_tree1")
    import pprint as pp
    data = np.zeros((cam_height, cam_width), dtype=float)
    generator = np.random.default_rng(seed=secrets.randbits(128))
    data[:,3:5] = tuple(generator.uniform(0.31, 0.35, (cam_height, 2)))
    data = data.reshape((cam_width * cam_height, 1), order="F")
    # log.warning(f"data:\n{data}")

    point_cloud = penv.deproject_pixels_to_points(data=data)


    # log.info(f"Point mask: {point_mask}")
    # time.sleep(10)
    # penv.deactivate_tree(tree_id_str="LPy_envy_tree1")
    # time.sleep(3)

    return


if __name__ == "__main__":
    main()
