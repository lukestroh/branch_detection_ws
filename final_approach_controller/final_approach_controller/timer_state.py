#!/usr/bin/env python3

from enum import Enum


class TimerState(Enum):
    RUNNING = 1
    STOPPING = 2
    STOPPED = 3
