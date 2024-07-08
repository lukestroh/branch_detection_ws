#!/usr/bin/env python3
"""
linear_slider_analysis.py
Luke Strohbehn
"""

import sqlite3 as sql
import pandas as pd
import numpy as np
import plotly.graph_objects as go

from controller_manager_msgs.msg import ControllerState
from rclpy.serialization import deserialize_message

import os

bag_dir = os.path.join(os.path.expanduser("~"), "bags", "jtc2")
db_path = os.path.join(bag_dir, "jtc2_0.db3")

def main():
    con = sql.connect(db_path)
    print(con)
    
    res = con.execute("select * from messages")
    # res = con.execute("select * from sqlite_master where type='table';")
    
    cols = [column[0] for column in res.description]
    df = pd.DataFrame.from_records(data=res.fetchall(), columns=cols)
    
    for i, _msg in enumerate(df["data"]):
        msg = deserialize_message(
            serialized_message=_msg,
            message_type=ControllerState
        )
        print(msg)
    return



if __name__ == "__main__":
    main()