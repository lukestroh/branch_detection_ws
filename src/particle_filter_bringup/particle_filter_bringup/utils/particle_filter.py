#!/usr/bin/env python3
"""
particle_filter.py
Luke Strohbehn

Adapted from TODO: get Jostan's repo

Section 5 of the datasheet mentions that there is a 25 degree Field of View (FOV), so we should include some noise in the other dimensions.
"""
from teensy32_tof_bringup.utils.parameters import Parameters
import numpy as np
import secrets
from dataclasses import dataclass

from particle_filter_bringup.utils.particle import Particle

import rclpy
from rclpy.impl import rcutils_logger

from typing import Any, Union


@dataclass
class ParticleFilterConfig(Parameters):
    # All ranges displayed in camera orientation format
    num_particles: int
    x_range: np.ndarray
    y_range: np.ndarray
    z_range: np.ndarray
    roll_range: np.ndarray = np.array([-np.pi, np.pi])
    pitch_range: np.ndarray = np.array([-np.pi, np.pi])
    yaw_range: np.ndarray = np.array([-np.pi, np.pi])


class ParticleFilter:
    def __init__(self, seed: int = None, environment: str = None) -> None:
        self.logger = rcutils_logger.RcutilsLogger(name="tof_particle_filter")
        self.particles: np.ndarray
        self.weights: np.ndarray
        self.best_particle: np.ndarray

        self.state: tuple

        # Numpy random generator
        if seed is None:
            self.seed = secrets.randbits(128)
        else:
            self.seed = seed
        self.generator = np.random.default_rng(seed=self.seed)

        # Load the tree model
        self.environment = self.load_model(name=environment)

        # Tof tfs

        return

    def load_model(self, name: str = None):
        """Load in the map or tree object if it exists"""
        if name is None:
            self.logger.info("No object model to load, running demo mode.")
        return

    def reset_filter(self, setup_data: ParticleFilterConfig):
        """Reset the filter with the provided setup_data"""
        num_particles = setup_data.num_particles
        self.x_range = setup_data.x_range
        self.y_range = setup_data.y_range
        self.z_range = setup_data.z_range
        self.roll_range = setup_data.roll_range
        self.pitch_range = setup_data.pitch_range
        self.yaw_range = setup_data.yaw_range
        self.particles = self.create_uniform_particles(
            x_range=self.x_range,
            y_range=self.y_range,
            z_range=self.z_range,
            N=num_particles,
        )
        self.weights = np.ones(num_particles) / num_particles
        return

    def get_measurement(self, state: tuple) -> None:
        """Update the measured state"""
        self.state = state
        return

    def create_uniform_particles(self, x_range, y_range, z_range, N) -> None:
        """
        @params
            x_range, y_range, z_range: tuple
                - ranges for possible locations
            N: int
                - number of particles to generate
        @returns
            np.ndarray(dtype=Particle)
        """
        particles = np.empty((N, 6))
        particles[:, 0] = self.generator.uniform(
            low=x_range[0], high=x_range[1], size=N
        )
        particles[:, 1] = self.generator.uniform(
            low=y_range[0], high=y_range[1], size=N
        )
        particles[:, 2] = self.generator.uniform(
            low=z_range[0], high=z_range[1], size=N
        )
        particles[:, 3] = self.generator.uniform(low=-1, high=1, size=N)
        particles[:, 4] = self.generator.uniform(low=-1, high=1, size=N)
        particles[:, 5] = self.generator.uniform(low=-1, high=1, size=N)

        # normalize orientation vector
        particles[:, 3:] = np.divide(particles[:, 3:], np.array([np.linalg.norm(particles[:, 3:], axis=1)]).T)


        _particles = np.empty(N, dtype=Particle)
        for i in range(N):
            _particles[i] = Particle()
            _particles[i].pose = particles[i]

        return _particles

    def create_gaussian_particles(
        self, mean: Union[list, np.ndarray], std: Union[list, np.ndarray], N: int
    ) -> np.ndarray:
        """
        @params
            mean: Union[list, np.ndarray]
            std: np.ndarray
            N: int
        @returns
            np.ndarray
        """
        particles = np.empty((self.N, 6))
        particles[:, 0] = mean[0] + (self.generator.standard_normal(size=N, dtype=float) * std[0])
        particles[:, 1] = mean[1] + (self.generator.standard_normal(size=N, dtype=float) * std[1])
        particles[:, 2] = mean[2] + (self.generator.standard_normal(size=N, dtype=float) * std[2])
        particles[:, 3] = mean[3] + (self.generator.standard_normal(size=N, dtype=float) * std[3])
        particles[:, 4] = mean[4] + (self.generator.standard_normal(size=N, dtype=float) * std[4])
        particles[:, 5] = mean[5] + (self.generator.standard_normal(size=N, dtype=float) * std[5])
        # self.particles[:, 3] = mean[3] + (self.generator.standard_normal(size=N, dtype=float) * std[3])

        _particles = np.empty(N, dtype=Particle)
        for i in range(N):
            _particles[i] = Particle()
            _particles[i].pose = particles[i]

        return _particles

    def predict(self, u, std, dt=0.1):
        """Move according to control input u with noise"""
        N = len(self.particles)
        # Update heading (tool0 coordinates)
        self.particles[:, 3] += u[0] + (
            self.generator.standard_normal(N) * std[0]
        )  # TODO: we have ee movement in/out in z direction...

        # Move in the commanded direction
        dist = (u[1] * dt) + (self.generator.standard_normal(N) * std[1])
        self.particles[:, 0] += (
            dist * self.particles[:, 3]
        )  # This used to represent the heading (hdg) range, but we have rpy now...
        self.particles[:, 1] += dist * self.particles[:, 3]
        self.particles[:, 2] += dist * self.particles[:, 3]

        # self.logger.info(f"{self.particles}")
        return
    
    def get_particle_scores(self, particles: np.ndarray, tof_data: np.ndarray):

        # Initialize scores
        scores = np.ones(particles.shape[0], dtype=float)

        # Calculate distance between sensed branch and the model tree for each Particle
        for i in range(particles.shape[0]):
            
            # Find the nearest neighbor of each branch in the tree
            distances, idx = ... # self.kd_tree.query(particle_coords)

            # find the range and bearing of sensed branch relative to the Particle
            range_diff = ...

            bearing_diff = ...

            prob_range = self.probability_of_values(range_diff)
            prob_bearing = self.probability_of_values(bearing_diff)

            # Update the scores
            scores *= prob_range * prob_bearing



        return
    
    def probability_of_values(self, arr, mean, std_dev):
        """Find the probability of a value in an array given a mean and standard deviation."""
        norm_pdf = (1 / (std_dev * np.sqrt(2 * np.pi))) * np.exp(
            -((arr - mean) ** 2) / (2 * std_dev ** 2)
        )
        return norm_pdf

    def tof_update(self, tof_tfs: np.ndarray, tof_data: np.ndarray) -> None:
        """Handle the tof readings. Update the associated weights of the positions"""

        # TODO: What are we comparing this to? The loaded model?
        # TODO: Get tof with histogram model?

        # tf2 transform particles.
        if tof_data is not None:
            tof0 = tof_data[0]
            tof1 = tof_data[1]

            # calculate position of branch on the model
            # i think i already have that in particles

            scores = self.get_particle_scores()
            self.weights = scores / np.linalg.norm(scores)


        return
    
    def motion_update(self):
        # See predict() ? 
        """Propogate the particles forward in time using TODO: servoing?"""
        return
    
    def update(self):
        # predict()
        # motion_update()
        # tof_update()
        return

        
    def estimate_distribution(self):
        """Update the mean and variance of the weighted particles"""
        # states = self.particles[:, 0:3]
        # self.particles_mean = np.average(states, weights=self.weights, axis=0)
        # self.particles_var = np.average(
        #     (states - self.particles_mean) ** 2, weights=self.weights, axis=0
        # )
        return

    def resample(self):
        """Resample the particle population based on the particle weights"""
        return


def plot_particles(data: np.ndarray):
    import plotly.graph_objects as go

    fig = go.Figure()

    for particle in data:
        fig.add_cone(
            x=[particle.pose[0]],
            y=[particle.pose[1]],
            z=[particle.pose[2]],
            u=[particle.pose[3]],
            v=[particle.pose[4]],
            w=[particle.pose[5]],
            sizemode="raw",
            sizeref=0.1,
            anchor="center",
    #         showlegend=False
        )

    fig.update_layout(scene_camera_eye=dict(x=-0.76, y=1.8, z=0.92))
    fig.update_traces(showscale=False) # gets rid of the default color bar on the side
    fig.show()

    return


def main():
    pfilter_conf = ParticleFilterConfig(
        num_particles=22,
        x_range=np.array([-0.1, 0.1]),
        y_range=np.array([-0.1, 0.1]),
        z_range=np.array([-0.1, 0.1]),
    )
    pfilter = ParticleFilter()
    pfilter.reset_filter(setup_data=pfilter_conf)

    plot_particles(data=pfilter.particles)

    pfilter.predict()

    

    tof_reading = np.array([0.256, 0.265])
    pfilter.update()
    # after the run
    return


if __name__ == "__main__":
    main()
