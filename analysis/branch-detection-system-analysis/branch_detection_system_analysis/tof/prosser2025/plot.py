#!/usr/bin/env python3

import glob
import os

from .bag_reader import BagReader

__here__ = os.path.dirname(os.getcwd())

print(__here__)


def get_data(location: str, tree_type: str):
    glob.glob(__here__)
