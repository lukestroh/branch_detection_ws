#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import h5py as hpy
from scipy.spatial.transform import Rotation
import plotly.graph_objects as go

import branch_detection_system_analysis.plot.plotly_helpers as ph
import branch_detection_system_analysis.plot.ray_tracing as rt

__here__ = os.path.dirname(__file__)


def get_data():

    start_poses = pd.read_hdf(f"{__here__}/data/start_poses.h5")

    coefs = np.array(hpy.File(f"{__here__}/data/coefs.h5").get("coefs"))

    quadratic_curve_df = pd.read_hdf(f"{__here__}/data/quadratic_curve.h5")
    t_vals = quadratic_curve_df['t_vals'].to_numpy()
    u_min, u_max = t_vals.min() - 1e-6, t_vals.max() + 1e-6

    generated_pt_scores = np.zeros(shape=len(start_poses), dtype=float)
   
    ################################################3
    positions = []
    localized = []
    aligned = []
    
    branch_radius = 0.0065
    plot_t_vals = np.linspace(sorted(t_vals)[0], sorted(t_vals)[-1], len(t_vals))
    tube_mesh_info = ph.get_tube_mesh_info(coefs=coefs, radius=branch_radius, u_vals=plot_t_vals)
    vertices, faces = rt.mesh_dict_to_arrays(tube_mesh_info)

    fig = go.Figure()
    fig = ph.plot_tube_mesh(fig=fig, tube_mesh_info=tube_mesh_info, name='branch')
    
    for i, (trial_name, pose) in enumerate(start_poses.iterrows()):
        print(f"{i}: {trial_name}")
        # if i != 20:
        #     continue
        # print(trial_name)
        rot_mat = Rotation.from_quat([pose["qx"][0], pose["qy"][0], pose["qz"][0], pose["qw"][0]]).as_matrix()
        
        ori_vec = rot_mat @ [0, 0, 1]
        ori_vec /= np.linalg.norm(ori_vec)
        pos = [pose["x"][0], pose["y"][0], pose["z"][0]]
        positions.append(pos)

        sensor_fov_deg = 18
        sigma_deg = 18 / 3

        rotated_fov_pts = rt.generate_cylindrical_pts(
            r_range=(0.04891, 0.04891),
            theta_range=(0, 2 * np.pi),
            z_range=(0, 0),
            num_r_pts=1,
            num_theta_pts=30,
            num_z_pts=1,
            start_point=pos,
            start_orientation=ori_vec,
        )

        # fig = go.Figure(
        #     data = go.Scatter3d(
        #         x=rotated_fov_pts[:, 0],
        #         y=rotated_fov_pts[:, 1],
        #         z=rotated_fov_pts[:, 2],
        #         mode="markers+text",
        #         text=[str(i) for i in range(len(rotated_fov_pts))],  # label = index
        #         textposition="top center"
        #     )
        # )
        # fig.show()
        
        rotated_pt_scores = np.zeros(shape=len(rotated_fov_pts), dtype=float)

        for j, rotated_pt in enumerate(rotated_fov_pts):
            # if j != 12:
            #     continue
            v_vec = np.cross(ori_vec, [0, 0, 1])
            w_vec = np.cross(ori_vec, v_vec)
            sampled_directions = rt.sample_gaussian_cone(
                u=ori_vec, v=v_vec, w=w_vec, sensor_fov_deg=sensor_fov_deg, sigma_deg=sigma_deg, num_samples=200
            )

            # scored_directions = rt.score_quadratic_directions(
            #     start_point=pos,
            #     directions=sampled_directions,
            #     coefs=coefs,
            #     branch_radius=0.0065,
            #     lambda_dist=0.25,
            #     u_min=u_min,
            #     u_max=u_max,
            # )
            scored_directions = rt.score_quadratic_directions2(
                start_point=pos,
                directions=sampled_directions,
                coefs=coefs,
                t_vals=t_vals,
                branch_radius=0.0065,
                mesh_vertices=vertices,
                mesh_faces=faces,
                tube_mesh_info=tube_mesh_info,
                lambda_dist=0.25,
                u_min=u_min,
                u_max=u_max,
            )
            
            # print(f'{j}: {np.mean(scored_directions)}')
            if not np.any(scored_directions):
                # print(scored_directions)
                direction_lines = rotated_pt + sampled_directions * 0.1
                """
                # fig = go.Figure()
                # plot_t_vals = np.linspace(sorted(t_vals)[0], sorted(t_vals)[-1], len(t_vals))
                # tube_mesh_info = ph.get_tube_mesh_info(coefs=coefs, radius=0.0065, u_vals=plot_t_vals)
                # fig = ph.plot_tube_mesh(fig=fig, tube_mesh_info=tube_mesh_info, name='branch')
                # fig.add_trace(
                #     go.Scatter3d(
                #         x=[pos[0]],
                #         y=[pos[1]],
                #         z=[pos[2]],
                #         name='start pt'
                #     )
                # )
                # fig.add_trace(
                #     go.Scatter3d(
                #         x=rotated_fov_pts[:, 0],
                #         y=rotated_fov_pts[:, 1],
                #         z=rotated_fov_pts[:, 2],
                #         mode='markers',
                #         showlegend=False
                #     )
                # )
                # for i, pt in enumerate(direction_lines):
                #     fig.add_trace(
                #         go.Scatter3d(
                #             x=[rotated_pt[0], direction_lines[i, 0]],
                #             y=[rotated_pt[1], direction_lines[i, 1]],
                #             z=[rotated_pt[2], direction_lines[i, 2]],
                #             mode='lines',
                #             showlegend=False
                #         )
                #     )
                # fig.update_layout(scene=dict(aspectmode='data'))
                # fig.show()
                """

            rotated_pt_scores = np.mean(scored_directions)

            print(rotated_pt_scores)

        generated_pt_scores[i] = np.mean(rotated_pt_scores)

        if not np.any(rotated_pt_scores):
            """
            # print(sampled_directions)
            # fig.add_trace(
            #     go.Scatter3d(
            #         x=[pos[0]],
            #         y=[pos[1]],
            #         z=[pos[2]],
            #     )
            # )
            # start_pts_repeated = np.tile(pos, (len(sampled_directions), 1))
            # new_ps = start_pts_repeated + 0.1 * sampled_directions
            # print(start_pts_repeated)
            # print(new_ps)
            # fig.add_trace(
            #     go.Scatter3d(
            #         x=[start_pts_repeated[:, 0], new_ps[:, 0]],
            #         y=[start_pts_repeated[:, 1], new_ps[:, 1]],
            #         z=[start_pts_repeated[:, 2], new_ps[:, 2]],
            #         mode='lines'
            #     )
            # )
            """
            pass

    generated_pt_scores = np.sqrt(generated_pt_scores)
    
    with hpy.File(f"{__here__}/data/generated_pt_scores.h5", "w") as f:
        f.create_dataset("generated_pt_scores", data=generated_pt_scores)

    positions = np.asarray(positions)
    fig.add_trace(
        go.Scatter3d(
            x=positions[:, 0],
            y=positions[:, 1],
            z=positions[:, 2],
            mode='markers',
            marker=dict(color='#000000')
        )
    )
    fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    print(generated_pt_scores)
    print(len(generated_pt_scores))
    print(np.where(generated_pt_scores != 0))

    return


def main():

    get_data()

    return


if __name__ == "__main__":
    main()