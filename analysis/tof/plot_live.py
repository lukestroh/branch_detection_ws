#!/usr/bin/env python3
import numpy as np
import plot_matplotlib as p
import serial_reader as sr
import matplotlib.pyplot as plt

import sys
import time


def main():
    ser = sr.create_connection()
    tof_array_size = (8,8)
    zone_data = np.zeros(tof_array_size, dtype=int)

    fig, ax, cax = p.create_fig(tof_array_size)

    start_time = time.time()


    while True:
        try:
            stream_data = sr.read_stream(ser)
            if stream_data is None:
                continue
            parsed = sr.parse_line(stream_data)
            if parsed is None:
                continue


            p.update_data_arr(data=zone_data, idx=parsed["zone"], value=parsed["distance"])

            if time.time() - start_time >= 0.01:

                p.update_fig(ax=ax, zone_data=zone_data)
                start_time = time.time()

        except KeyboardInterrupt:
            sys.exit()

    return

if __name__ == "__main__":
    main()
