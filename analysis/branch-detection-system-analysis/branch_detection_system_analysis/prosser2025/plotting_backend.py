#!/usr/bin/env python3
import glob
import numpy as np
import os


"""
Data collection functions
"""
def get_files_by_trial_name(warehouse_path: str, name: str) -> list[str]:
    files = glob.glob(os.path.join(warehouse_path, name+"_0") + "/*.h5")
    return files


def get_files_by_topic(warehouse_path: str, topic: str):
    return glob.glob(warehouse_path + f"/**/*{topic}*.h5")


"""
Filtering functions
"""
def filter_files_by_trial_number(files: list[str], trial_number: int) -> list[str]:
    return [file for file in files if file.endswith(f"{str(trial_number).zfill(3)}.h5")]




"""
Plotting functions
"""