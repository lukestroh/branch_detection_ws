#!/usr/bin/env python3

import pprint as pp
import pandas as pd


class DummyPublisher:
    def __init__(
        self,
    ):
        return

    def publish(self, msg=None):
        return


class DummyNode:
    def __init__(self):
        self.info = lambda x: pp.pprint(x)
        self.debug = lambda x: pp.pprint(x)
        self.warn = lambda x: pp.pprint(x)
        self.error = lambda x: pp.pprint(x)
        self.fatal = lambda x: pp.pprint(x)

        self._param_far_plane_filter = 0.25
        self._param_robot_eef_part = "mock_pruner"
        self._param_robot_base_part = "amiga"

        self._pub_windowed_data = DummyPublisher()

        return
