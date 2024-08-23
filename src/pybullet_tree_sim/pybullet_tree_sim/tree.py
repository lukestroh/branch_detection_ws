#!/usr/bin/env python3
from __future__ import annotations

"""
tree.py
authors: Abhinav Jain, Luke Strohbehn

Generates a tree in PyBullet
"""
from collections import defaultdict
import glob
import math
import os
import pickle
from typing import Optional, Tuple, List
import secrets
import numpy as np
import pybullet
import pywavefront
from nptyping import NDArray, Shape, Float
from pybullet_tree_sim import RGB_LABEL, URDF_PATH
from scipy.spatial.transform import Rotation
import xacro
import xml

# from pruning_sb3.pruning_gym.helpers import roundup, rounddown
# from memory_profiler import profile


class TreeException(Exception):
    pass


class Tree:
    """This class is used to create a tree object by loading the urdf file and the obj file
    along with the labelled obj file. This class is used to filter the points on the tree and create a curriculum of points
    to be used in training.

    To create a tree object, the following parameters are required:
    urdf_path: The path to the urdf file
    obj_path: The path to the obj file
    labelled_obj_path: The path to the labelled obj file
    pos: The position of the tree
    orientation: The orientation of the tree
    scale: The scale of the tree
    """

    _tree_xacro_path = os.path.join(URDF_PATH, "trees", "envy", "tree.urdf.xacro")

    def __init__(
        self,
        id: int,
        type: str,
        urdf_path: str,
        obj_path: str,
        # labelled_obj_path: str,
        position=np.array([0, 0, 0]),
        orientation=np.array([0, 0, 0, 1]),
        scale: float = 1.0,

        randomize_pose=False,
        verbose: bool = True,
        seed: int | None = None
    ) -> None:
        # Set seed
        if seed is not None:
            self.seed = seed
        else:
            self.seed = secrets.randbits(128)
        self.generator: np.random.Generator = np.random.default_rng(seed=self.seed)
        self.verbose = verbose

        # Required paths to make tree
        self.urdf_path = urdf_path
        # self.labelled_obj_path = labelled_obj_path
        self.obj_path = obj_path

        self.rgb_label = RGB_LABEL
        # Load the tree object
        tree_obj = pywavefront.Wavefront(obj_path, create_materials=True, collect_faces=True)
        self.urdf: xml.dom.minidom.Document
 
        # labelled_tree_obj = pywavefront.Wavefront(labelled_obj_path, create_materials=True, collect_faces=True)

        # Tree specific parameters
        self.scale = scale
        self.id = id,
        self.type = type
        self.init_pos = position
        self.init_orientation = orientation

        # Set tree pose
        if randomize_pose:
            new_pos, new_orientation = self._randomize_pose()
        else:
            new_pos = position
            new_orientation = orientation

        self.pos = new_pos
        self.orientation = new_orientation

        # # Variables to store the vertices and statistics of the tree
        # self.vertex_and_projection = []
        # self.projection_mean = np.array(0.0)
        # self.projection_std = np.array(0.0)
        # self.projection_sum_x = np.array(0.0)
        # self.projection_sum_x2 = np.array(0.0)
        # self.reachable_points = []

        # # Label textured tree
        # vertex_to_label = self.label_vertex_by_color(self.rbg_label, tree_obj.vertices, labelled_tree_obj.vertices)

        # # append the label to each vertex
        # tree_obj_vertices_labelled = []
        # for i, vertex in enumerate(tree_obj.vertices):
        #     tree_obj_vertices_labelled.append(vertex + (vertex_to_label[vertex],))

        # # TODO: begin() method or something
        # # self.transformed_vertices = list(map(lambda x: self.transform_obj_vertex(x, pyb), tree_obj_vertices_labelled))

        # # if pickled file exists load and return
        # path_component = os.path.normpath(self.urdf_path).split(os.path.sep)
        # if not os.path.exists("./pkl/" + str(path_component[3])):
        #     os.makedirs("./pkl/" + str(path_component[3]))
        # pkl_path = "./pkl/" + str(path_component[3]) + "/" + str(path_component[-1][:-5]) + "_points.pkl"

        # if os.path.exists(pkl_path):
        #     self.load_points_from_pickle(pkl_path)
        # else:
        #     # Get all points on the tree
        #     self.get_all_points(tree_obj)
        #     self.filter_outliers()
        #     self.filter_trunk_points()
        #     # self.filter_points_below_base()

        #     # dump reachable points to file using pickle
        #     with open(pkl_path, "wb") as f:
        #         pickle.dump((self.pos, self.orientation, self.vertex_and_projection), f)

        #     if self.verbose > 0:
        #         print(f"INFO: Number of points: {len(self.vertex_and_projection)}")
        #         print(f"INFO: Saved points to pickle file {pkl_path}")

        # # Make different meshes for each label
        # # make bins
        # self.or_bins = self.create_bins(18, 36)
        # # Go through vertex_and_projection and assign to bins
        # self.populate_bins(self.vertex_and_projection)

        # del self.vertex_and_projection

        return

    def _randomize_pose(self) -> tuple:
        # TODO: Randomize position to bounds?
        new_position = np.array([0, 0, 0])
        # TODO: Multiply orientation with initial orientation
        new_orientation = pybullet.getQuaternionFromEuler(
            self.generator.uniform(low=-1, high=1, size=(3,)) * np.pi / 180 * 5
        )
        return new_position, new_orientation

    # @staticmethod
    # def create_bins(num_latitude_bins, num_longitude_bins):
    #     """
    #     Create bins separated by 10 degrees on a unit sphere.

    #     Parameters:
    #         num_latitude_bins (int): Number of bins along the latitude direction.
    #         num_longitude_bins (int): Number of bins along the longitude direction.

    #     Returns:
    #         list of tuples: List of tuples where each tuple represents a bin defined by
    #                         (latitude_min, latitude_max, longitude_min, longitude_max).
    #     """
    #     bin_size = np.deg2rad(10)  # Convert degrees to radians
    #     bins = {}
    #     for i in range(num_latitude_bins):
    #         lat_min = np.rad2deg(-np.pi / 2 + i * bin_size)
    #         lat_max = np.rad2deg(-np.pi / 2 + (i + 1) * bin_size)
    #         for j in range(num_longitude_bins):
    #             lon_min = np.rad2deg(-np.pi + j * bin_size)
    #             lon_max = np.rad2deg(-np.pi + (j + 1) * bin_size)
    #             bins[(round((lat_min + lat_max) / 2), round((lon_min + lon_max) / 2))] = []
    #     return bins

    # def populate_bins(self, points):
        """
        Populate the bins based on a list of direction vectors.

        Parameters:
            direction_vectors (list of numpy.ndarray): List of direction vectors.
            bins (list of tuples): List of bins where each tuple represents a bin defined by
                                   (latitude_min, latitude_max, longitude_min, longitude_max).

        Returns:
            list of lists: List of lists where each sublist represents the indices of direction vectors
                           assigned to the corresponding bin.
        """
        offset = 1e-3
        for i, point in enumerate(points):

            direction_vector = point[1]
            direction_vector = direction_vector / np.linalg.norm(direction_vector)
            lat_angle = np.rad2deg(np.arcsin(direction_vector[2])) + offset
            lon_angle = np.rad2deg(np.arctan2(direction_vector[1], direction_vector[0])) + offset
            lat_angle_min = rounddown(lat_angle)
            lat_angle_max = roundup(lat_angle)
            lon_angle_min = rounddown(lon_angle)
            lon_angle_max = roundup(lon_angle)
            bin_key = (round((lat_angle_min + lat_angle_max) / 2), round((lon_angle_min + lon_angle_max) / 2))
            # if bin_key[0] not in between -85 and 85 set as 85 or -85
            # if bin_keyp[1] not in between -175 and 175 set as 175 or -175

            if bin_key[0] > 85:
                bin_key = (85, bin_key[1])
            elif bin_key[0] < -85:
                bin_key = (-85, bin_key[1])
            if bin_key[1] > 175:
                bin_key = (bin_key[0], 175)
            elif bin_key[1] < -175:
                bin_key = (bin_key[0], -175)
            self.or_bins[bin_key].append(
                (self.urdf_path, point, self.orientation, self.scale)
            )  # Add collision meshes for each tree here

    # def label_vertex_by_color(self, labels, unlabelled_vertices, labelled_vertices):
    #     # create a dictionary of vertices and assign label using close enough vertex on labelled tree obj
    #     vertex_to_label = {}
    #     for i, vertex in enumerate(unlabelled_vertices):
    #         vertex_to_label[vertex] = None

    #     for j, labelled_vertex in enumerate(labelled_vertices):
    #         min_dist = 100000
    #         for i in labels.keys():
    #             # assign label that is closest
    #             dist = np.linalg.norm(np.array(labelled_vertex[3:]) - np.array(i))
    #             if dist < min_dist:
    #                 min_dist = dist
    #                 vertex_to_label[labelled_vertex[:3]] = labels[i]
    #     return vertex_to_label

    # def load_points_from_pickle(self, pkl_path):
    #     with open(pkl_path, "rb") as f:
    #         data = pickle.load(f)
    #         self.pos = data[0]
    #         self.orientation = data[1]
    #         self.vertex_and_projection = data[2]
    #     if self.verbose > 0:
    #         print(f"INFO: Loaded points from pickle file {pkl_path}")
    #         print(f"INFO: Number of points: {len(self.vertex_and_projection)}")

    # def transform_obj_vertex(self, vertex, pyb):
        vertex_pos = np.array(vertex[0:3]) * self.scale
        vertex_orientation = [0, 0, 0, 1]  # Dont care about orientation
        vertex_w_transform = pyb.con.multiplyTransforms(self.pos, self.orientation, vertex_pos, vertex_orientation)
        return np.array(vertex_w_transform[0]), vertex[3]

    # def get_all_points(self, tree_obj):
    #     for num, face in enumerate(tree_obj.mesh_list[0].faces):
    #         # Order the sides of the face by length
    #         ab = (
    #             face[0],
    #             face[1],
    #             np.linalg.norm(self.transformed_vertices[face[0]][0] - self.transformed_vertices[face[1]][0]),
    #         )
    #         ac = (
    #             face[0],
    #             face[2],
    #             np.linalg.norm(self.transformed_vertices[face[0]][0] - self.transformed_vertices[face[2]][0]),
    #         )
    #         bc = (
    #             face[1],
    #             face[2],
    #             np.linalg.norm(self.transformed_vertices[face[1]][0] - self.transformed_vertices[face[2]][0]),
    #         )

    #         normal_vec = np.cross(
    #             self.transformed_vertices[ac[0]][0] - self.transformed_vertices[ac[1]][0],
    #             self.transformed_vertices[bc[0]][0] - self.transformed_vertices[bc[1]][0],
    #         )
    #         # Only front facing faces
    #         if np.dot(normal_vec, [0, 1, 0]) < 0:
    #             continue
    #         # argsort sorts in ascending order
    #         sides = [ab, ac, bc]
    #         sorted_sides = np.argsort([x[2] for x in sides])
    #         ac = sides[sorted_sides[2]]
    #         ab = sides[sorted_sides[1]]
    #         bc = sides[sorted_sides[0]]
    #         # |a
    #         # |\
    #         # | \
    #         # |  \
    #         # |   \
    #         # |    \
    #         # b______\c
    #         perpendicular_projection = compute_perpendicular_projection_vector(
    #             self.transformed_vertices[ac[0]][0] - self.transformed_vertices[ac[1]][0],
    #             self.transformed_vertices[bc[0]][0] - self.transformed_vertices[bc[1]][0],
    #         )

    #         scale = np.random.uniform()
    #         tree_point = (1 - scale) * self.transformed_vertices[ab[0]][0] + scale * self.transformed_vertices[ab[1]][0]

    #         # Label the face as the majority label of the vertices
    #         labels = [
    #             self.transformed_vertices[ab[0]][1],
    #             self.transformed_vertices[ab[1]][1],
    #             self.transformed_vertices[ac[0]][1],
    #             self.transformed_vertices[ac[1]][1],
    #             self.transformed_vertices[bc[0]][1],
    #             self.transformed_vertices[bc[1]][1],
    #         ]

    #         # If all three vertices are the same label, assign that label
    #         # else assign label "JOINT"
    #         if len(set(labels)) == 1:
    #             label = labels[0]
    #         else:
    #             label = "JOINT"
    #         if label != "SPUR" and label != "WATER_BRANCH":
    #             continue
    #         self.vertex_and_projection.append((tree_point, perpendicular_projection, normal_vec, label))
    #         # This projection mean is used to filter corner/flushed faces which do not correspond to a branch
    #         self.projection_sum_x += np.linalg.norm(perpendicular_projection)
    #         self.projection_sum_x2 += np.linalg.norm(perpendicular_projection) ** 2
    #     self.projection_mean = self.projection_sum_x / len(self.vertex_and_projection)
    #     self.projection_std = np.sqrt(
    #         self.projection_sum_x2 / len(self.vertex_and_projection) - self.projection_mean ** 2
    #     )

    # def filter_outliers(self):
    #     # Filter out outliers
    #     print("Number of points before filtering: ", len(self.vertex_and_projection))
    #     self.vertex_and_projection = list(
    #         filter(
    #             lambda x: np.linalg.norm(x[1]) > self.projection_mean + 0.5 * self.projection_std,
    #             self.vertex_and_projection,
    #         )
    #     )
    #     print("Number of points after filtering: ", len(self.vertex_and_projection))

    # def filter_points_below_base(self, base_xyz):
    #     # Filter out points below the base of the arm
    #     self.vertex_and_projection = list(filter(lambda x: x[0][2] > base_xyz[2], self.vertex_and_projection))

    # def filter_trunk_points(self):
        self.vertex_and_projection = list(
            filter(lambda x: abs(x[0][0] - self.pos[0]) > 0.8, self.vertex_and_projection)
        )
        print("Number of points after filtering trunk points: ", len(self.vertex_and_projection))

    # def is_reachable(
    #     self, vertex: Tuple[NDArray[Shape["3, 1"], Float], NDArray[Shape["3, 1"], Float]], base_xyz: NDArray, env, pyb
    # ) -> bool:
    #     if vertex[3] != "SPUR":
    #         return False
    #     ur5_base_pos = np.array(base_xyz)

    #     # Meta condition
    #     dist = np.linalg.norm(ur5_base_pos - vertex[0], axis=-1)

    #     if dist >= 0.98:
    #         return False

    #     j_angles = env.ur5.calculate_ik(vertex[0], None)
    #     env.ur5.set_joint_angles(j_angles)
    #     for _ in range(100):
    #         pyb.con.stepSimulation()
    #     ee_pos, _ = env.ur5.get_current_pose(env.ur5.end_effector_index)
    #     dist = np.linalg.norm(np.array(ee_pos) - vertex[0], axis=-1)
    #     if dist <= 0.03:
    #         return True

    #     return False

    # def get_reachable_points(self, env, pyb):
    #     self.reachable_points = list(filter(lambda x: self.is_reachable(x, env, pyb), self.vertex_and_projection))
    #     np.random.shuffle(self.reachable_points)
    #     print("Number of reachable points: ", len(self.reachable_points))
    #     if len(self.reachable_points) < 1:
    #         print("No points in reachable points", self.urdf_path)
    #         # self.reset_tree()

    #     return self.reachable_points

    # @staticmethod
    # def make_trees_from_folder(
    #     trees_urdf_path: str,
    #     trees_obj_path: str,
    #     trees_labelled_path: str,
    #     pos: NDArray,
    #     orientation: NDArray,
    #     scale: int,
    #     num_points: int,
    #     num_trees: int,
    #     randomize_pose: bool = False,
    # ) -> List:
        trees: List[Tree] = []
        for urdfs, objs in zip(
            sorted(glob.glob(trees_urdf_path + "/*.urdf")), sorted(glob.glob(trees_obj_path + "/*.obj"))
        ):
            print(urdfs, objs, labelled_obj)
            if len(trees) >= num_trees:
                break
            trees.append(
                Tree(
                    urdf_path=urdfs,
                    obj_path=objs,
                    pos=pos,
                    orientation=orientation,
                    scale=scale,
                    # num_points=num_points,
                    randomize_pose=randomize_pose,
                )
            )

        return trees

    @staticmethod
    def _load_urdf(urdf_path: str, mappings: dict | None = None) -> str:
        urdf_content = xacro.process_file(input_file_name=Tree._tree_xacro_path, mappings=mappings)
        return urdf_content

    @staticmethod
    def get_tree_urdf(
        scale: float,
        tree_id: int = None,
        tree_type: str = None,
        # pos: NDArray,
        # orientation: NDArray,
        tree_urdf_path: str | None = None,
        namespace: str = "",
        parent: str = "world",
        position: str = "0.0 0.0 0.0",
        orientation: str = "0.0 0.0 0.0",
    ) -> xml.dom.minidom.Document:
        """Create a tree using some input data. Trees can also be generated from a hard-coded URDF, but that isn't recommended. If `tree_urdf_path` is not None, then load that URDF. Otherwise, process an xacro file with some given input parameters."""

        if tree_urdf_path is not None:
            _tree_urdf_path = tree_urdf_path
            urdf_mappings = None
        else:
            # If there is no path, process an xacro file
            _tree_urdf_path = Tree._tree_xacro_path
            urdf_mappings = {
                "namespace": namespace,
                "tree_id": str(tree_id),
                "tree_type": tree_type,
                "parent": parent,
                # "xyz": position,
                # "rpy": orientation,
            }
        urdf_content = Tree._load_urdf(urdf_path=_tree_urdf_path, mappings=urdf_mappings).toprettyxml()
        return urdf_content

    @staticmethod
    def create_tree(
        scale: float,
        tree_id: int = None,
        tree_type: str = None,
        # pos: NDArray,
        # orientation: NDArray,
        tree_urdf_path: str | None = None,
        namespace: str = "",
        parent: str = "world",
        position: np.ndarray = np.zeros(3, dtype=float),
        orientation: Rotation = Rotation.from_euler('xyz', np.zeros(3, dtype=float))
    ) -> Tree:
        quat = orientation.as_quat()
        # rpy_orientation =
        print("Orientation: ", quat.tostring().decode())
        tree = Tree()
        tree.urdf = Tree.get_tree_urdf(scale=scale, tree_id=tree_id, tree_type=tree_type, tree_urdf_path=tree_urdf_path)

        return tree



def main():
    # env: The environment object
    # pyb: The pybullet object
    # urdf_path: The path to the urdf file
    # obj_path: The path to the obj file
    # labelled_obj_path: The path to the labelled obj file

    from pyb_environment import PyBEnvironment

    env = PyBEnvironment()

    return


if __name__ == "__main__":
    main()
