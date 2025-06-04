#!/usr/bin/env python3
import branch_detection_system_analysis.plot.plotting_backend as pb
from collections import defaultdict
import os
import pandas as pd
from pathlib import Path
import pprint as pp

import rclpy.logging

logger = rclpy.logging.get_logger("bin_by_category")


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
bags_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection")
warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

def iter_grouped_metadata(grouped_metadata: dict): 
    for datetime_str, topics in grouped_metadata.items():
        group = {}
        for topic, metadata in topics.items():
            group[topic] = pd.read_hdf(metadata['filename'])
        yield datetime_str, group

def main():
    # 20250229
    # 20250230
    files = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250530")
    filtered_files = pb.filter_files_by_topics(
        files=files,
        topics=[
            "fbrw_controller_alignment_success",
            "fbrw_controller_localization_success",
        ],
    )

    metadata_cache = {Path(file).name: pb.extract_file_metadata(file) for file in filtered_files}

    grouped_metadata = pb.group_metadata(list(metadata_cache.values()), "datetime", "topic")

    data_iterator = iter_grouped_metadata(grouped_metadata=grouped_metadata)
    
    binned_results = {
        'success': 0,
        'failure': 0,
        'aligned_no_localization': 0,
        'localized_no_alignment': 0
    }
    dataless_quantity = 0
    undetermined_names = []

    for datetime, data_dict in data_iterator:
        localization_success = None
        alignment_success = None
        for topic_name, topic_df in data_dict.items():
            if topic_df.empty:
                dataless_quantity += 1
                continue
            if topic_name == "fbrw_controller_alignment_success":
                alignment_success = topic_df.at[0, 'controller_success']
            if topic_name == "fbrw_controller_localization_success":
                localization_success = topic_df.at[0, 'controller_success']
        
        if alignment_success and localization_success:
            binned_results['success'] += 1
        elif (alignment_success and not localization_success):
            binned_results['aligned_no_localization'] += 1
            undetermined_names.append(datetime)

        elif (not alignment_success and localization_success):
            binned_results['localized_no_alignment'] += 1
            
        elif not alignment_success and not localization_success:
            binned_results["failure"] += 1
        
    print(dataless_quantity)    
    print(binned_results)
    print(undetermined_names)


    
    return


if __name__ == "__main__":
    main()
