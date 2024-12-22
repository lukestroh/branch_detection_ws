#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as ani

import pandas as pd

from bag_reader import BagReader
import os
import numpy as np
import glob
from typing import List

def get_databases() -> List[str]:
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    bag_dir = os.path.join(pkg_dir, "bags")
    dbs = glob.glob(bag_dir + "/vl53l8cx_10cm_2024*/*.db3")
    return dbs

def get_df_from_db(database: str):

    br = BagReader(bag_file=database)
    metadata = list(br.query(topic_name='/microROS/vl53l8cx/distance'))
    # print(list(data[0][1].data))

    timestamps = [d[0] for d in metadata]
    sensor_data = [list(d[1].data) for d in metadata]

    df = pd.DataFrame(data=list(zip(timestamps, sensor_data)), columns=['timestamp', 'distances'])

    # print(df['distances'][0])
    return df



def create_fig(array_size: tuple):
    vmin=100
    vmax = 140
    fig, ax = plt.subplots()
    cax = ax.imshow(np.zeros(array_size), cmap="viridis_r", vmin=vmin, vmax=vmax)
    cbar = fig.colorbar(cax)
    cbar.set_ticks(ticks=[vmin, (vmin + vmax)/2, vmax])
    plt.draw()
    return fig, ax, cax




def animate(fig, ax, cax, dataframe):

    def _update_animation(i):
        cax.set_array(np.asarray(dataframe['distances'][i]).reshape((8,8)))
        return
    
    animation = ani.FuncAnimation(
        fig=fig,
        func=_update_animation,
        repeat=True,
        frames=len(dataframe['distances']),
        interval=1/15 * 1000

    )

    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    writer = ani.PillowWriter(
        fps=15,
    )
    animation.save(
        filename=os.path.join(pkg_dir, "analysis/test.gif"),
        writer=writer,
    )

    return



def main():

    dbs = get_databases()
    
    df = get_df_from_db(database=dbs[2])
    fig, ax, cax = create_fig(array_size=(8,8))
    animate(fig, ax, cax, dataframe=df)
    # fig.show()


    return



if __name__ == "__main__":
    main()