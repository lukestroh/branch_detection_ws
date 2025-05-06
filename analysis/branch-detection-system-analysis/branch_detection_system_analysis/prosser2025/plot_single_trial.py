#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates
from branch_detection_system_analysis.prosser2025 import curve_fitting as cf
from branch_detection_system_analysis.prosser2025 import plotly_helpers as ph
from branch_detection_system_analysis.prosser2025 import plotting_backend as pb
import glob
import numpy as np
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import plotly.subplots
import os
from scipy.spatial.transform import Rotation
import scipy.optimize as so
import traceback




ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse")




    # file = files[0]
    # return file


def main():
    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name="bds__arm_farm__20250430_17-10-31")
    
    return


if __name__ == "__main__":
    main()
