#!/usr/bin/env python3


"""
https://medium.com/@mashruf.zaman/reading-ros2-db3-rosbag-file-in-python-76566521ce3d
https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/

"""

import sqlite3 as sql
import pandas as pd
import plotly.graph_objects as go
import numpy as np

from teensy32_tof_msgs.msg import ToFData
from rclpy.serialization import deserialize_message

import os

bag_dir = "/home/luke/bags/tof_100mm_calibrated"

def get_data():
    db_path = os.path.join(bag_dir, "tof_100mm_calibrated_0.db3")

    con = sql.connect(db_path)

    # res = con.execute("select * from sqlite_master where type='table';")
    res_raw = con.execute("select * from messages where topic_id=4") # Get the raw data (topic_id=4)
    res_filtered = con.execute("select * from messages where topic_id=2") # Get the filtered data (topic_id=1)
    # res = con.execute("tables")
    # print(res.fetchall())
    # res = con.execute("select * from '/tof_filtered'")
    cols_raw = [column[0] for column in res_raw.description]
    cols_filtered = [column[0] for column in res_filtered.description]
    df = pd.DataFrame.from_records(data=res_raw.fetchall(), columns=cols_raw)
    df_filtered = pd.DataFrame.from_records(data=res_filtered.fetchall(), columns=cols_filtered)
    for i, msg in enumerate(df['data']):
        _msg = deserialize_message(msg, ToFData)
        df.at[i, "tof0"] = _msg.tof0
        df.at[i, "tof1"] = _msg.tof1

    for i, msg in enumerate(df_filtered['data']):
        _msg = deserialize_message(msg, ToFData)
        df_filtered.at[i, "tof0_filtered"] = _msg.tof0
        df_filtered.at[i, "tof1_filtered"] = _msg.tof1

    return df, df_filtered


def plot(data):
    fig = go.Figure()

    raw_data, filtered_data = data

    t = (raw_data["timestamp"] - raw_data["timestamp"][0]) / 1e9
    t_filtered = (filtered_data["timestamp"] - filtered_data["timestamp"][0]) / 1e9

    fig.add_trace(
        go.Scatter(
            x=t,
            y=raw_data["tof0"],
            name="tof0 raw"
        )
    )
    fig.add_trace(
        go.Scatter(
            x=t,
            y=raw_data["tof1"],
            name="tof1 raw"
        )
    )
    fig.add_trace(
        go.Scatter(
            x=t_filtered,
            y=filtered_data["tof0_filtered"],
            name="tof0 filtered"
        )
    )
    fig.add_trace(
        go.Scatter(
            x=t_filtered,
            y=filtered_data["tof1_filtered"],
            name="tof1 filtered"
        )
    )
    print("std0: ", np.std(raw_data["tof0"]))
    print("std1: ", np.std(raw_data["tof1"]))

    return fig


def main():
    data = get_data()
    fig = plot(data=data)
    fig.show()
    
    return


if __name__ == "__main__":
    main()