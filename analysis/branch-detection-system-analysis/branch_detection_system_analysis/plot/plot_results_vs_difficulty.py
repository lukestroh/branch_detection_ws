#!/usr/bin/env python3
import pandas as pd
import h5py as hpy
import numpy as np
import os
import plotly.graph_objects as go
import plotly.colors as plc
import plotly.express as px

color_dict = {"easy": "#52C742", "medium": "#efa524", "hard": "#fa1b1b"}


def main():

    __here__ = os.path.dirname(__file__)

    print(__here__)

    with hpy.File(f"{__here__}/data/coefs.h5", "r") as f:
        coefs = f.get("coefs")[:]

    with hpy.File(f"{__here__}/data/rt_scores.h5", "r") as f:
        rt_scores = f.get("rt_scores")[:]

    df_branch_info = pd.read_hdf(f"{__here__}/data/branch_info.h5")
    df_successes = pd.read_hdf(f"{__here__}/data/successes.h5")
    df_success_poses = pd.read_hdf(f"{__here__}/data/success_poses.h5")
    df_undetermined_poses = pd.read_hdf(f"{__here__}/data/undetermined_poses.h5")

    print(len(df_successes))
    print(df_branch_info)

    df_localized = pd.read_hdf(f"{__here__}/data/localized.h5")
    df_aligned = pd.read_hdf(f"{__here__}/data/aligned.h5")

    print(df_aligned.loc[df_aligned["aligned"] == True])

    zero_scores = rt_scores[rt_scores == 0]
    non_zero_scores = rt_scores[rt_scores != 0]
    print(zero_scores)
    print(len(zero_scores))
    print(non_zero_scores)
    print(len(non_zero_scores))
    # import sys
    # sys.exit()
    # print(non_zero_scores)
    # min_val = np.min(non_zero_scores)
    # max_val = np.max(non_zero_scores)
    min_val = 0.00
    max_val = 0.80

    zero_score_idxs = np.where(rt_scores == 0)[0]
    non_zero_score_idxs = np.where(rt_scores != 0)[0]

    sliced_successes = df_successes.iloc[non_zero_score_idxs].to_numpy().flatten()

    n_bins = 8
    bins = np.linspace(min_val, max_val, n_bins + 1)
    # counts, _ = np.histogram(non_zero_scores, bins=bins)

    # bin_idx = np.searchsorted(bins, non_zero_scores, side='right') - 1
    # binned_successes = [sliced_successes[bin_idx == i] for i in range(n_bins)]

    # bin_percent_success = []

    # for _bin in binned_successes:
    #     ones = np.ones(len(_bin))
    #     zeros = np.zeros(len(_bin))
    #     try:
    #         percent_success = np.where(_bin == 2, ones, zeros).mean() * 100
    #     except RuntimeWarning:
    #         percent_success = 0.0
    #     bin_percent_success.append(percent_success)
    # # print(bin_percent_success)

    # # easy_successes = sliced_successes[: counts[0]].flatten()
    # # medium_successes = sliced_successes[counts[0] : (counts[0] + counts[1])].flatten()
    # # hard_successes = sliced_successes[(counts[0] + counts[1]) :].flatten()

    # # ones = np.ones(len(easy_successes))
    # # zeros = np.zeros(len(easy_successes))
    # # percent_easy = np.where(easy_successes == 2, ones, zeros).mean() * 100

    # # ones = np.ones(len(medium_successes))
    # # zeros = np.zeros(len(medium_successes))
    # # percent_medium = np.where(medium_successes == 2, ones, zeros).mean() * 100

    # # ones = np.ones(len(hard_successes))
    # # zeros = np.zeros(len(hard_successes))
    # # percent_hard = np.where(hard_successes == 2, ones, zeros).mean() * 100

    # # bin_centers = [(bins[i] + bins[i + 1]) for i in range(len(counts))]
    # bin_centers = 0.5 * (bins[:-1] + bins[1:])
    # print(bin_centers)
    # # bin_widths = [(bins[i+1]-bins[i])*1.0 for i in range(len(counts))],
    # bin_widths = np.diff(bins)
    # bin_labels = ["Easy", "Medium", "Hard"]

    new_bins = np.concatenate([bins[:-2], bins[-1:]])  # edges: e0..e_{m-2}, e_m

    # recompute histogram and binned groups with the new edges
    counts, _ = np.histogram(non_zero_scores, bins=new_bins)
    bin_idx = np.searchsorted(new_bins, non_zero_scores, side="right") - 1
    binned_successes = [sliced_successes[bin_idx == i] for i in range(len(new_bins) - 1)]

    # compute percent_success robustly (avoid warnings on empty bins)
    bin_percent_success = []
    for _bin in binned_successes:
        if len(_bin) == 0:
            bin_percent_success.append(0.0)
        else:
            bin_percent_success.append(((_bin == 2).mean()) * 100)

    # centers/widths for plotting
    bin_centers = 0.5 * (new_bins[:-1] + new_bins[1:])
    bin_widths = np.diff(new_bins)

    # custom tick labels (make last one "<0.6" or whatever you want)
    tickvals = bin_centers
    ticktext = [f"{c:.2f}" for c in bin_centers]
    ticktext[-1] = "<0.6"  # replace label for the merged last bin

    marker_line_width = 1
    marker_line_color = ("black",)

    # percents = [percent_easy, percent_medium, percent_hard]

    base_scale = None
    for name in ("matter_r", "Matter_r"):
        base_scale = getattr(px.colors.sequential, name, None)
        if base_scale:
            break
    if base_scale is None:
        # fallback to a different sequential palette if 'matter' isn't present
        base_scale = px.colors.sequential.Viridis

    # sample the continuous colorscale at n_bins positions -> gives one color per bin
    positions = [i / (n_bins - 1) if n_bins > 1 else 0.5 for i in range(n_bins)]
    palette = plc.sample_colorscale(base_scale, positions)  # returns color strings

    fig = go.Figure()
    fig.add_trace(
        go.Bar(
            x=bin_centers,
            y=bin_percent_success,
            width=bin_widths,
            # ids=['Easy', "Medium", "Hard"]
            # marker_color=[color_dict["easy"], color_dict["medium"], color_dict["hard"]],
            marker_color=palette,
            marker_line_color=marker_line_color,
            marker_line_width=marker_line_width,
            # text=counts,  # numbers to show on bars
            # texttemplate="%{text}",  # how to format the text (here: show counts)
            # textposition="outside",  # put them above each bar
            # textfont=dict(size=20),  # tweak if needed
        )
    )

    fig.update_layout(
        xaxis=dict(
            title=dict(text="Visibility score", font=dict(size=28, color="#000000")),
            tickfont=dict(
                size=20,
                color='#000000'
            ),
            tickvals=tickvals,
            tickformat=".2f",
            ticktext=ticktext,
        ),
        yaxis=dict(
            title=dict(
                text="Percent success",
                font=dict(size=28, color="#000000"),
            ),
            tickfont=dict(
                size=20,
                color="#000000"
            ),
        ),
    )
    fig.show()

    return


if __name__ == "__main__":
    main()
