#!/usr/bin/env python3

from bag_reader import BagReader
import os
import numpy as np
import glob
import pandas as pd
import plotly.graph_objs as go

from typing import List



def get_databases() -> List[str]:
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    bag_dir = os.path.join(pkg_dir, "bags")
    dbs = glob.glob(bag_dir + "/vl6180*/*.db3")
    return dbs
    
def get_df_from_db(database: str):
    br = BagReader(bag_file=database)
    metadata = list(br.query(topic_name='/vl6180/filtered'))
    timestamps = [d[0] / 1e9 for d in metadata]
    tof_data = [list(d[1].data) for d in metadata]
    tof0_data = [d[0] for d in tof_data]
    tof1_data = [d[1] for d in tof_data]
    
    data = {
        'timestamp': timestamps,
        'tof0': tof0_data,
        'tof1': tof1_data
    }
    
    df = pd.DataFrame(data)
    return df
    
def save_df_to_csv(df: pd.DataFrame, filename: str):
    df.to_csv(filename, index=False)
    return
    
def plot_df(df: pd.DataFrame):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=df['timestamp'], y=df['tof0'], mode='lines', name='tof0'))
    fig.add_trace(go.Scatter(x=df['timestamp'], y=df['tof1'], mode='lines', name='tof1'))
    fig.show()
    return
    
def main():
    dbs = get_databases()
    df = get_df_from_db(dbs[0])
    save_df_to_csv(df, 'csv/tof_data.csv')
    plot_df(df)
    return
    
    
    
    
if __name__ == "__main__":
    main()