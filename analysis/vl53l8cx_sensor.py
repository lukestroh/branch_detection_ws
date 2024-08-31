#!/usr/bin/env python3
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import pandas as pd
import glob
import numpy as np
import os

from vl53l8cx import plot_matplotlib as pm

from bag_reader import BagReader

from typing import List


topic_name = "/microROS/vl53l8cx/distance"

def get_databases() -> List[str]:
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    bag_dir = os.path.join(pkg_dir, "bags")
    dbs = glob.glob(bag_dir + "/vl53l8cx*/*.db3")
    return dbs


def get_df_from_db(database: str):
    bag_reader = BagReader(bag_file=database)

    # print(bag_reader.topics)

    vl53l8cx_distances = list(bag_reader.query(topic_name=topic_name))

    # print(vl53l8cx_distances)
    time_raw = [point[0] for point in vl53l8cx_distances]
    time_raw = [(t - time_raw[0]) / 1e9 for t in time_raw]

    distances_raw = [np.resize(point[1].data, (8,8)) for point in vl53l8cx_distances]

    data_raw = {
        "time_raw": time_raw,
        "vl53l8cx_distances_raw": distances_raw
    }

    df_raw = pd.DataFrame(data=data_raw)
    # df_filtered = pd.DataFrame(data=data_filtered)
    return df_raw


def plot_frame(data: pd.DataFrame):
    fig = go.Figure()   
    fig.add_heatmap(
        z=data["vl53l8cx_distances_raw"][79],
        coloraxis="coloraxis"
    )

    # Set color axis limits
    fig.update_layout(
        coloraxis=dict(
            cmin=90,
            cmax=115
        )
    )
    
    print(data["vl53l8cx_distances_raw"][0])

    fig.update_layout(coloraxis_showscale=False)

    fig.show()
    return

def plot_animation(data: pd.DataFrame) -> None:
    tof_array_size = (8,8)
    fig, ax, cax = pm.create_fig(tof_array_size)

    for i in range(data.shape[0]):
        pm.update_fig(ax=ax, zone_data=data["vl53l8cx_distances_raw"][i])
        fig.show()

    print(data.shape[0])
    return


def main():
    dbs = get_databases()
    dfs = []
    for db in dbs:
        dfs.append(get_df_from_db(database=db))
        # plot_frame(data=df)

    plot_animation(data=dfs[0])


    

    return

if __name__ == "__main__":
    main()