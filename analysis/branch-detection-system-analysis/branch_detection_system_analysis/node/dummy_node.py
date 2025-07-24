#!/usr/bin/env python3

import pprint as pp

class DummyPublisher():
    def __init__(self, ):
        return
    
    def publish(self, msg = None):
        return

class DummyNode():
    def __init__(self):
        self.info = lambda x: pp.pprint(x)
        self.debug = lambda x: pp.pprint(x)
        self.warn = lambda x: pp.pprint(x)
        self.error = lambda x: pp.pprint(x)
        self.fatal = lambda x: pp.pprint(x)

        self.filter_far_plane = 0.25

        self._pub_windowed_data = DummyPublisher()

        return