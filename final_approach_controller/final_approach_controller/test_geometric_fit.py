#!/usr/bin/env python3
import plotly.graph_objects as go
import curve_fitting as cf

import numpy as np

from scipy.optimize import minimize

coefs = [0.9619636258510362, 0.3683743914120028, 0]
r = 0.4893

a = coefs[0]
b = coefs[1]
c = coefs[2]


fit = minimize(cf.parabola, x0=0, args=(a,b,c))

theta_min = fit.x
y_min = cf.parabola(x=theta_min, a=a, b=b, c=c)
print(f'theta_min: {theta_min}')
print(f'y_min: {y_min}')

h_root = (r**2 - r * (2 * a * theta_min + b)) / a
print(f'h_root: {h_root}')

# We want to find where y(theta_min + delta) = y_min + h
delta = h_root / r
print(f'delta: {delta}')

Delta = delta * 2
print(f"Delta: {Delta}")

theta_min_plus_h = theta_min + h_root
print(f"t + h: {theta_min_plus_h}")

y_at_tmin_plus_delta = cf.parabola(x=theta_min+delta, a=a, b=b, c=c)
print(f'y(t+d): {y_at_tmin_plus_delta}')

h_from_calc = y_at_tmin_plus_delta - y_min
print(f'h_from_calc: {h_from_calc}')

print()

assert np.isclose(h_root, h_from_calc, atol=1e-9)


def extra():
    import numpy as np
    import plotly.graph_objects as go

    def cylinder(r, h, a =0, nt=100, nv =50):
        """
        parametrize the cylinder of radius r, height h, base point a
        """
        theta = np.linspace(0, 2*np.pi, nt)
        v = np.linspace(a, a+h, nv )
        theta, v = np.meshgrid(theta, v)
        x = r*np.cos(theta)
        y = r*np.sin(theta)
        z = v
        print(v)
        return x, y, z

    def boundary_circle(r, h, nt=100):
        """
        r - boundary circle radius
        h - height above xOy-plane where the circle is included
        returns the circle parameterization
        """
        theta = np.linspace(0, 2*np.pi, nt)
        x= r*np.cos(theta)
        y = r*np.sin(theta)
        z = h*np.ones(theta.shape)
        return x, y, z
    r1 = 2
    a1 = 0
    h1 = 5
    r2 = 1.35
    a2 = 1
    h2 = 3

    x1, y1, z1 = cylinder(r1, h1, a=a1)
    x2, y2, z2 = cylinder(r2, h2, a=a2)

    colorscale = [[0, 'blue'],
                [1, 'blue']]

    cyl1 = go.Surface(x=x1, y=y1, z=z1,
                    colorscale = colorscale,
                    showscale=False,
                    opacity=0.5)
    xb_low, yb_low, zb_low = boundary_circle(r1, h=a1)
    xb_up, yb_up, zb_up = boundary_circle(r1, h=a1+h1)

    bcircles1 =go.Scatter3d(x = xb_low.tolist()+[None]+xb_up.tolist(),
                            y = yb_low.tolist()+[None]+yb_up.tolist(),
                            z = zb_low.tolist()+[None]+zb_up.tolist(),
                            mode ='lines',
                            line = dict(color='blue', width=2),
                            opacity =0.55, showlegend=False)

    cyl2 = go.Surface(x=x2, y=y2, z=z2,
                    colorscale = colorscale,
                    showscale=False,
                    opacity=0.7)

    xb_low, yb_low, zb_low = boundary_circle(r2, h=a2)
    xb_up, yb_up, zb_up = boundary_circle(r2, h=a2+h2)

    bcircles2 =go.Scatter3d(x = xb_low.tolist()+[None]+xb_up.tolist(),
                            y = yb_low.tolist()+[None]+yb_up.tolist(),
                            z = zb_low.tolist()+[None]+zb_up.tolist(),
                            mode ='lines',
                            line = dict(color='blue', width=2),
                            opacity =0.75, showlegend=False)

    layout = go.Layout(scene_xaxis_visible=False, scene_yaxis_visible=False, scene_zaxis_visible=False)
    fig =  go.Figure(data=[cyl2, bcircles2, cyl1, bcircles1], layout=layout)

    fig.update_layout(scene_camera_eye_z= 0.55)
    fig.layout.scene.camera.projection.type = "orthographic" #commenting this line you get a fig with perspective proj

    fig.show()

    return


def plot():
    fig = go.Figure()
    thetas = np.linspace(-np.pi, np.pi, 1000)
    ys = cf.parabola(thetas, a, b, c)
    
    fig.add_trace(
        go.Scatter(
            x=thetas,
            y=ys,
        )
    )
    fig.add_hline(y=y_at_tmin_plus_delta[0], line_dash="dot", line_color="green",
              annotation_text="Threshold", annotation_position="top left")

    return fig

def main():
    
    fig = plot()
    fig.show()
    return

if __name__ == "__main__":
    main()
