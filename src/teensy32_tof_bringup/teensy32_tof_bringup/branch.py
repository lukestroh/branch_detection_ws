#!/usr/bin/env python3
from __future__ import annotations

import numpy as np

from typing import List, Union, Dict, Any


class Branch():
    def __init__(self, name: str = "") -> None:
        # Parent/child branches
        self._name: str = name
        self._children: List[Union[Branch, None]] = []
        self._parent: Union[Branch, None] = None

        # Branch geometry
        self._t_value: float = None  # where along 0-1 index of the parent branch is this branch
        self._phi: float = None # rotation from the z-axis with respect to the parent branch
        self._theta: float = None # rotation around the z-axis with respect to the parent branch
        
        
        return
    
    def info(self) -> Dict[str, Any]:
        """Generate a dict that contains the information of a branch"""
        if self._parent is None:
            parent = None
        else:
            parent = self._parent.name
        info_dict = dict(
            name=self._name,
            parent=parent,
            t=self._t_value,
            phi=self._phi,
            theta=self._theta,
        )
        return info_dict
    
    @property
    def name(self) -> str:
        """Return branch name"""
        return self._name
    
    @name.setter
    def name(self, name: str) -> None:
        """Set branch name"""
        self._name = name
        return
    
    @property
    def children(self) -> List[Branch]:
        """Return a list of branch children"""
        return self._children

    def add_child(self, child: Branch, t_value: float, phi: float, theta: float) -> None:
        """Add a child branch object"""
        parent = dict(
            branch=self,
            t_value=t_value,
            phi=phi,
            theta=theta
        )
        child.parent = parent
        self._children.append(child)
        return

    @property
    def parent(self) -> Dict[str, Any]:
        """Get the parent branch information"""
        return self._parent

    @parent.setter
    def parent(self, parent: dict) -> None:
        """Set the parent branch of the current branch
        @params
            parent (dict): A dictionary containing the parent Branch object
                and the offset from the parent branch to the self branch.
                parent = {
                    "branch": Branch,
                    "t_value": float,
                    "phi": float,
                    "theta": float
                }
        @returns
            None
        """
        try:
            self._parent = parent["branch"]
            self._t_value = parent["t_value"]
            self._phi = parent["phi"]
            self._theta = parent["theta"]
        except KeyError as e:
            print(e)
        return
    
    @property
    def t_value(self):
        """Get t-value with respect to the parent"""
        return self._t_value
    
    @t_value.setter
    def t_value(self, t_value: float):
        """Set t-value with respect to the parent"""
        if t_value < 0:
            raise ValueError(f"t value cannot be less than zero. Value was {t_value}.")
        self._t_value = t_value
        return

    @property
    def phi(self):
        """Get angle phi with respect to the parent"""
        return self._phi
    
    @phi.setter
    def phi(self, phi: float):
        """Set angle phi with respect to the parent"""
        if phi > np.pi or phi < -np.pi:
            raise ValueError("Angle phi cannot be greater than π or less than -π.")
        self._phi = phi
        return
    
    @property
    def theta(self):
        """Get angle theta with respect to the parent"""
        return self._theta

    @theta.setter
    def theta(self, theta: float):
        """Set angle theta with respect to the parent"""
        if theta > np.pi or theta < -np.pi:
            raise ValueError("Angle theta cannot be greater than π or less than -π.")
        self._theta = theta
        return

    def evaluate_branch_fitness(self):
        return

    def project_2d_image(self, camera_matrix):
        return

    def get_2d_image_pts(self, camera, pts):
        """Get the set of points from the image from reprojection"""
        return


def main():
    from objprint import op
    trunk = Branch(name="trunk")
    branch0 = Branch(name="branch0")
    branch00 = Branch(name="branch0.0")
    trunk.add_child(branch0, t_value=0.1, phi=0.12, theta=1.15)
    branch0.add_child(branch00, t_value=0.24, phi=0.18, theta=0.75)

    op(trunk)

    return


if __name__ == "__main__":
    main()