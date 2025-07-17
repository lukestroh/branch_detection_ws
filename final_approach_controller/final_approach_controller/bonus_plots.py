fig = go.Figure()
fig.add_trace(
    go.Scatter(
        x=wrist_3_data,
        y=fpf_data["data"],
        mode="markers",
        text=fpf_data["ts"],
        hovertemplate="theta: %{x}<br>d: %{y}<br>time: %{text}<extra></extra>",
    )
)
fig.show()

# r = fpf_data['tof_arm_radius']
# x = r * np.cos(fpf_data['wrist_state'])
# y = r * np.sin(fpf_data['wrist_state'])
# z = fpf_data['data']
# fig = go.Figure()
# fig.add_trace(
#     go.Scatter3d(
#         x=x[10:-50],
#         y=y[10:-50],
#         z=z[10:-50],

#         mode="markers",
#         text=fpf_data['ts'],
#         hovertemplate="theta: %{x}<br>d: %{y}<br>time: %{text}<extra></extra>"

#     )
# )
# fig.show()
