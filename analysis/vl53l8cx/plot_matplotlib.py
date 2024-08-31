#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import numpy as np

VMAX=135
VMIN=95

def create_fig(array_size: tuple):
    fig, ax = plt.subplots()

    cax = ax.imshow(np.zeros(array_size), cmap="viridis_r", vmin=VMIN, vmax=VMAX)
    cbar = fig.colorbar(cax)
    cbar.set_ticks(ticks=[0, VMIN, VMAX])
    plt.draw()
    return fig, ax, cax

def update_fig(ax, zone_data: np.ndarray):
    cax = ax.imshow(zone_data, cmap="viridis_r", vmin=VMIN, vmax=VMAX)
    # img.colorbar()
    plt.draw()
    # print(zone_data)
    plt.pause(1/15)
    return

def update_data_arr(data: np.ndarray, idx: int, value: int):
    row = idx // data.shape[0]
    col = idx % data.shape[1]
    data[row, col] = value
    return data

def animate_fig(data: np.ndarray, animate:bool, repeat:bool, frames: int, interval: int):


    return


def main():
    data_size = (8,8)
    
    fig = create_fig(data_size)

    while True:
        update_fig()
        fig.show()
    return

if __name__ == "__main__":
    main()

