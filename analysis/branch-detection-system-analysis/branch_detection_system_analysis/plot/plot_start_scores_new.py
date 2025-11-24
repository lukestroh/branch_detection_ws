#!/usr/bin/env python3
# import branch_detection_system_analysis.plot.plotting_backend as pb
import branch_detection_system_analysis.plot.plotly_helpers as ph
import branch_detection_system_analysis.plot.plot_start_points_new_score as psp


import numpy as np
import plotly.graph_objects as go
import plotly.io as pio


def make_cylindrical_plot():
    z_axis = [0, 0, 1]
    branch_center = np.array([0.47, 0.17, 1.47])  # redo from real start point?
    branch_orientation = np.array([-0.5, 1, 0.1])
    branch_radius = 0.005
    branch_length = 0.5

    r_range = (0.01, 0.12)
    theta_range = (-np.pi, np.pi)
    z_range = (0, -0.2)
    num_r_pts = 20
    num_theta_pts = 50
    num_z_pts = 20

    start_pts_orientation = np.cross(z_axis, branch_orientation)
    start_pts_center = np.asarray(branch_center) + start_pts_orientation * 0.1

    fig = psp.plot_cylindrical_start_points(
        r_range=r_range,
        theta_range=theta_range,
        z_range=z_range,
        num_r_pts=num_r_pts,
        num_theta_pts=num_theta_pts,
        num_z_pts=num_z_pts,
        start_point=start_pts_center,
        start_orientation=start_pts_orientation,
        branch_center=branch_center,
        branch_orientation=branch_orientation,
        branch_radius=branch_radius,
        branch_length=branch_length,
    )

    fig = ph.plot_cylinder(
        center=branch_center,
        orientation=branch_orientation,
        radius=branch_radius,
        height=branch_length,
        name="branch",
        color="#856957",
        fig=fig,
    )

    fig = ph.plot_3d_coordinate_frame(
        fig=fig,
        position=np.asarray([0, 0.4, 1.3]),
        orientation=np.identity(3),
        cone_scale=0.5,
        axis_length=0.1,
        name="xyz",
        parent_frame="xyz",
    )

    fig.update_layout(
        # paper_bgcolor='rgba(0,0,0,0)',
        # plot_bgcolor='rgba(0,0,0,0)',
        scene=dict(
            xaxis=dict(backgroundcolor="rgba(0,0,0,0)", showbackground=False, visible=False),
            yaxis=dict(backgroundcolor="rgba(0,0,0,0)", showbackground=False, visible=False),
            zaxis=dict(backgroundcolor="rgba(0,0,0,0)", showbackground=False, visible=False),
        ),
        showlegend=False,
    )

    fig.show()

    return


def make_angles_plot():
    z_axis = [0, 0, 1]
    branch_center = np.array([0.47, 0.17, 1.47])  # redo from real start point?
    branch_orientation = np.array([-0.5, 1, 0.1])
    branch_radius = 0.005
    branch_length = 0.5

    start_pts_orientation = np.cross(z_axis, branch_orientation)
    start_pts_center = np.asarray(branch_center)

    roll_range = (0, 0)
    pitch_range = (-np.pi / 4, np.pi / 4)
    yaw_range = (-np.pi / 2, np.pi / 2)
    z_range = (-0.1, -0.25)

    num_roll_pts = 1
    num_pitch_pts = 100
    num_yaw_pts = 1
    num_z_pts = 20

    fig = psp.plot_radial_start_points(
        roll_range=roll_range,
        pitch_range=pitch_range,
        yaw_range=yaw_range,
        z_range=z_range,
        num_roll_pts=num_roll_pts,
        num_yaw_pts=num_yaw_pts,
        num_pitch_pts=num_pitch_pts,
        num_z_pts=num_z_pts,
        start_point=start_pts_center,
        start_orientation=start_pts_orientation,
        branch_center=branch_center,
        branch_orientation=branch_orientation,
        branch_radius=branch_radius,
        branch_length=branch_length,
    )

    fig = ph.plot_cylinder(
        center=branch_center,
        orientation=branch_orientation,
        radius=branch_radius,
        height=branch_length,
        name="branch",
        color="#856957",
        opacity=1.0,
        fig=fig,
    )

    fig.update_layout(
        scene=dict(
            xaxis=dict(backgroundcolor="rgba(0,0,0,0)", showbackground=False, visible=False),
            yaxis=dict(backgroundcolor="rgba(0,0,0,0)", showbackground=False, visible=False),
            zaxis=dict(backgroundcolor="rgba(0,0,0,0)", showbackground=False, visible=False),
        ),
        showlegend=False,
    )

    # pio.write_image(fig=fig, file="./fig.pdf", format="pdf")
    fig.show()

    return


if __name__ == "__main__":
    import time
    import os

    os.nice(10)

    start_time = time.process_time()
    # make_cylindrical_plot()
    make_angles_plot()
    end_time = time.process_time()

    print(f"Program took {end_time - start_time}s to run")
