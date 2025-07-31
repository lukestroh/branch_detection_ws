#!/usr/bin/env python3
from branch_detection_system_analysis.plot import plotting_backend as pb
import os
import pandas as pd
import pprint as pp


def main():
    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

    data_dict = {}
    files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250730") + pb.get_files_by_date(warehouse_path=warehouse_path, date="20250729")



    files_by_loc_by_topic = pb.filter_files_by_topic(files=files_by_date, topic='generated_start_poses')

    # pp.pprint(files_by_loc_by_topic)

    for file in files_by_loc_by_topic:
        df = pd.read_hdf(file)
        

    return


if __name__ == "__main__":
    main()
