#!/usr/bin/env python3
import sqlite3 as sql
import pandas as pd
import plotly.graph_objects as go
import glob
import os

from bag_reader import BagReader

from rclpy.serialization import deserialize_message
# from 

from teensy32_tof_msgs.msg import ToFData
from typing import List


def get_databases() -> List[str]:
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    bag_dir = os.path.join(pkg_dir, "bags")
    dbs = glob.glob(bag_dir + "/indoor*/*.db3")
    return dbs


def get_df_from_db(database: str):
    bag_reader = BagReader(bag_file=database)

    trial_name = ""

    # tof_data_raw = list(bag_reader.query(topic_name="/microROS/tof_data"))
    tof_data_raw = list(bag_reader.query(topic_name="/microROS/tof_data_array"))
    tof_data_filtered = list(bag_reader.query(topic_name="/tof_filtered"))

    time_raw = [point[0] for point in tof_data_raw]
    time_raw = [(t - time_raw[0]) / 1e9 for t in time_raw]
    tof0_raw = [point[1].data[0] for point in tof_data_raw]
    tof1_raw = [point[1].data[1] for point in tof_data_raw]

    time_filtered = [point[0] for point in tof_data_filtered]
    time_filtered = [(t - time_filtered[0]) / 1e9 for t in time_filtered]
    tof0_filtered = [point[1].data[0] for point in tof_data_filtered]
    tof1_filtered = [point[1].data[1] for point in tof_data_filtered]

    data_raw = {
        # "trial": trial_name,
        "time_raw": time_raw,
        "tof0_raw": tof0_raw,
        "tof1_raw": tof1_raw,
    }

    data_filtered = {
        "time_filtered": time_filtered,
        "tof0_filtered": tof0_filtered,
        "tof1_filtered": tof1_filtered
    }

    df_raw = pd.DataFrame(data=data_raw)
    df_filtered = pd.DataFrame(data=data_filtered)
    return df_raw, df_filtered


def plot_data(data: pd.DataFrame):
    fig = go.Figure()

    data_raw = data[0]
    data_filtered = data[1]

    fig.add_trace(
        go.Scatter(
            x=data_raw["time_raw"],
            y=data_raw["tof0_raw"],
            name="tof0_raw"
        )
    )
    fig.add_trace(
        go.Scatter(
            x=data_raw["time_raw"],
            y=data_raw["tof1_raw"],
            name="tof1_raw"
        )
    )
    fig.add_trace(
        go.Scatter(
            x=data_filtered["time_filtered"],
            y=data_filtered["tof0_filtered"],
            name="tof1_filtered"
        )
    )
    fig.add_trace(
        go.Scatter(
            x=data_filtered["time_filtered"],
            y=data_filtered["tof1_filtered"],
            name="tof1_filtered"
        )
    )
    fig.show()

    return fig

def main():
    
    dbs = get_databases()
    for db in dbs:
        dfs = get_df_from_db(database=db)
        plot_data(data=dfs)
        
    
    return


if __name__ == "__main__":
    main()