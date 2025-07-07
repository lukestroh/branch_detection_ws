import branch_detection_system_analysis.plot.plotting_backend as pb


def main():

    fig = pb.plot_cylinder(
        center=[0, 1, 1],
        orientation=[1, 1, 1],
        radius=0.005,
        height=0.2,
        name="branch",
        color="#856957",
    )

    fig.show()

    return


if __name__ == "__main__":
    main()
