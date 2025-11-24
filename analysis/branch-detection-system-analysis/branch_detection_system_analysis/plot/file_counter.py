#!/usr/bin/env python3
import numpy as np
import branch_detection_system_analysis.plot.plotting_backend as pb
import os
import pandas as pd
from datetime import datetime, date
import re
import glob
import pprint as pp

def filter_dirnames_by_date(dirnames: list[str], start_date, end_date=None):
    # Normalize inputs to datetime.date
    def to_date(d):
        if isinstance(d, date):
            return d
        return datetime.strptime(d, "%Y%m%d").date()

    start = to_date(start_date)
    end = to_date(end_date) if end_date else start


    filtered = []
    for d in dirnames:
        m = re.search(r"(\d{8})_\d{2}-\d{2}-\d{2}$", d)
        if not m:
            continue

        dt = datetime.strptime(m.group(1), "%Y%m%d").date()
        if start <= dt <= end:
            filtered.append(d)

    return filtered


def get_files():
    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    bags_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection"))

    dirs = [
        os.path.join(bags_path, name)
        for name in os.listdir(bags_path)
        if os.path.isdir(os.path.join(bags_path, name))
    ]
    filtered_dirs = [] 
    filtered_dirs.extend(filter_dirnames_by_date(dirnames=dirs, start_date="20250729")[26:])
    filtered_dirs.extend(filter_dirnames_by_date(dirnames=dirs, start_date="20250730")[:-6])

    html_dict = {}

    for _dir in filtered_dirs:
        html_files = glob.glob(os.path.join(_dir, "*.html"))
        html_dict[os.path.basename(_dir)] = html_files

    file_counts = np.zeros(shape=len(html_dict))
    
    for i, (name, files) in enumerate(html_dict.items()):
        file_counts[i] = len(files)

    print(file_counts)
    print(len(np.where(file_counts == 12)[0]))
    print(len(np.where(file_counts == 3)[0]))
    return 



def main():
    import pprint as pp
    files = get_files()
    
    print(files)
    return


if __name__ == "__main__":
    main()
    