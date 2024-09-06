#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import numpy as np

VMIN=70
VMAX=150

def create_fig(array_size: tuple):
    fig, ax = plt.subplots()

    cax = ax.matshow(np.zeros(array_size), cmap="viridis_r", vmin=VMIN, vmax=VMAX)
    fig.colorbar(cax, ax=ax)
    plt.draw()
    return fig, ax, cax

def update_fig(fig, ax, zone_data: np.ndarray):
    cax = ax.matshow(zone_data, cmap="viridis_r", vmin=VMIN, vmax=VMAX)
        
    plt.draw()

    plt.pause(1/15)
    return

def update_data_arr(data: np.ndarray, idx: int, value: int):
    row = idx // data.shape[0]
    col = idx % data.shape[1]
    data[row, col] = value
    return data


def update_ani(frame, data, cax):
    cax.set_array(data.iloc[frame]["vl53l8cx_distances_raw"])
    return cax


def main():
    data_size = (8,8)
    
    fig = create_fig(data_size)

    while True:
        update_fig()
        fig.show()
    return

if __name__ == "__main__":
    main()

