#!/usr/bin/env python3
import plotly.graph_objects as go
import numpy as np

def create_fig(array_size: tuple) -> go.Figure:
    fig = go.Figure()
    fig.add_heatmap(
        z=np.zeros(array_size)
    )
    return fig

def create_fig_widget(array_size: tuple) -> go.FigureWidget:
    fig = create_fig(array_size)
    widget_fig = go.FigureWidget(fig)
    return widget_fig

def update_fig(data: np.ndarray) -> go.Figure:
    fig = go.Figure()
    fig.add_heatmap(
        z=data
    )
    return fig

def update_fig_widget(widget: go.FigureWidget):
    # fig = update_fig(fig)
    return

def update_data_arr(data: np.ndarray, idx: int, value: int):
    row = idx // data.shape[0]
    col = idx % data.shape[1]
    data[row, col] = value
    return data

def main():
    data_size = (8,8)
    data = np.zeros(data_size)


    widget_fig = create_fig_widget()
    return

if __name__ == "__main__":
    main()
