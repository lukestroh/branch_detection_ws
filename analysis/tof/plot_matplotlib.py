#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

def create_fig(array_size: tuple):
    fig, ax = plt.subplots()

    cax = ax.imshow(np.zeros(array_size), cmap="magma_r", vmin=0, vmax=2000)
    cbar = fig.colorbar(cax)
    cbar.set_ticks(ticks=[0, 500, 1000, 1500, 2000])
    plt.draw()
    return fig, ax, cax

def update_fig(ax, zone_data: np.ndarray):
    cax = ax.imshow(zone_data, cmap="magma_r", vmin=0, vmax=2000)
    # img.colorbar()
    plt.draw()
    # print(zone_data)
    plt.pause(0.001)
    return

def update_data_arr(data: np.ndarray, idx: int, value: int):
    row = idx // data.shape[0]
    col = idx % data.shape[1]
    data[row, col] = value
    return data


def main():
    data_size = (8,8)
    
    fig = create_fig(data_size)

    while True:
        update_fig()
        fig.show()
    return

if __name__ == "__main__":
    main()

