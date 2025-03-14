#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates
import glob
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import plotly.subplots
import os

import pprint as pp


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))


def get_files_by_iteration(trial_num: int) -> list[str]:
    trial_num_str = str(trial_num).zfill(3)
    return glob.glob(warehouse_path + f"/**/*{trial_num_str}.h5")


def get_files_by_topic(topic: str):
    return glob.glob(warehouse_path + f"/**/*{topic}*.h5")


def get_files_by_topics(topics: list[str]) -> list[str]:
    files = []
    for topic in topics:
        files += get_files_by_topic(topic=topic)
    return files


def get_files_by_topics_by_trial_name(topics: list[str], trial_name: str) -> list[str]:
    files = []
    for topic in topics:
        files += glob.glob(warehouse_path + f"/**/*{trial_name}*{topic}*.h5")
    return files


def main():
    # files_by_num = get_files_by_iteration(trial_num=3)
    # pp.pprint(files_by_num)
    # print()

    # files_by_topics = get_files_by_topics(topics=["joint_angles", 'tof0_filtered'])
    # pp.pprint(files_by_topic)

    files_by_topics_by_trial_name = get_files_by_topics_by_trial_name(
        topics=["joint_angles", "tof0_filtered", "tof1_filtered"], trial_name="t1.2.1"
    )
    pp.pprint(files_by_topics_by_trial_name)
    return


if __name__ == "__main__":
    main()
