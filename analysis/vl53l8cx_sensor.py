#!/usr/bin/env python3
from functools import partial
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import matplotlib.animation as ani
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
    dbs = glob.glob(bag_dir + "/*black*/*.db3")
    return dbs


def get_df_from_db(database: str) -> pd.DataFrame | None:
    bag_reader = BagReader(bag_file=database)

    if not bag_reader.topics:
        # print(bag_reader.topics)
        return None

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
        z=data["vl53l8cx_distances_raw"][0],
        coloraxis="coloraxis"
    )

    # Set color axis limits
    fig.update_layout(
        coloraxis=dict(
            cmin=250,
            cmax=315
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
        pm.update_fig(fig=fig, ax=ax, zone_data=data["vl53l8cx_distances_raw"][i])
        fig.show()

    print(data.shape[0])
    return

def create_animation(data: pd.DataFrame) -> ani.FuncAnimation:
    tof_array_size = (8,8)
    fig, ax, cax = pm.create_fig(tof_array_size)

    an = ani.FuncAnimation(
        fig=fig,
        func=partial(pm.update_ani, data=data, cax=cax),
        frames=len(data["time_raw"]),
        interval=1/15
    )

    return an

def save_animation(anim, filepath, filename):
    _filename = os.path.basename(filename).strip().split(".")[0] + ".gif"
    # anim.save(filename=os.path.join(filepath, _filename), writer="imagemagick")
    # plt.show()
    return


def get_stats(data: pd.DataFrame):
    arr3d = np.stack(data["vl53l8cx_distances_raw"].values, axis=0)
    # print(data["vl53l8cx_distances_raw"].std(axis=0))
    std = np.std(arr3d, axis=0)
    cov = np.multiply(std, std)
    mean = np.mean(arr3d, axis=0)

    print(f"Mean: {mean}")
    print(f"Std: {std}")
    print(f"Cov: {cov}")


    return

def main():
    import pprint


    __here__ = os.path.dirname(__file__)
    gifs_dir = os.path.join(__here__, "gifs")


    dbs = get_databases()
    dfs = []
    pprint.pprint(dbs)
    for db in dbs:
        df = get_df_from_db(database=db)
        if df is not None:
            dfs.append(df)

    INPUT = 0

    input(f"{dbs[INPUT]}")

    ani = create_animation(data=dfs[INPUT])
    save_animation(anim=ani, filepath=gifs_dir, filename=dbs[INPUT])

    get_stats(data=dfs[INPUT])


    return


if __name__ == "__main__":
    main()
