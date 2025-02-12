import numpy as np
from math import pi, sin, cos
from typing import List, Tuple

class TrajectoryGenerator:
    """
    A class responsible for generating trajectories for a 6-DOF system with prismatic (X, Y, Z) and 
    spherical (Roll, Pitch, Yaw) joints.

    Attributes:
    ----------
    duration : float
        Total duration of the trajectory.
    timestep : float
        Time step for the trajectory generation.
    frequency : float
        Frequency of motion (Hz).
    amplitude_translation : float
        Amplitude for prismatic joint motion.
    amplitude_rotation : float
        Amplitude for spherical joint rotation (radians).
    """

    def __init__(self, duration: float = 20.0, timestep: float = 0.01, frequency: float = 0.5, 
                 amplitude_translation: float = 0.5, amplitude_rotation: float = pi / 12):
        """
        Initializes the trajectory generator with given parameters.

        Parameters:
        ----------
        duration : float, optional
            Total duration of the trajectory (default: 20.0 seconds).
        timestep : float, optional
            Time step for the trajectory (default: 0.01 seconds).
        frequency : float, optional
            Frequency of motion (default: 0.5 Hz).
        amplitude_translation : float, optional
            Amplitude for X, Y, Z translation (default: 0.5 meters).
        amplitude_rotation : float, optional
            Amplitude for spherical joint rotation (default: π/12 radians).
        """
        self.duration = duration
        self.timestep = timestep
        self.frequency = frequency
        self.amplitude_translation = amplitude_translation
        self.amplitude_rotation = amplitude_rotation

    def generate_trajectory(self) -> Tuple[List[List[float]], List[List[float]], List[float]]:
        """
        Generates the trajectory for a 6-DOF system (X, Y, Z, Roll, Pitch, Yaw).

        Returns:
        -------
        joint_positions : List[List[float]]
            List of joint positions for each time step.
        joint_velocities : List[List[float]]
            List of joint velocities for each time step.
        timestamps : List[float]
            List of timestamps corresponding to the positions and velocities.
        """
        num_steps = int(self.duration / self.timestep)
        timestamps = []
        joint_positions = []
        joint_velocities = []

        for i in range(num_steps):
            t = i * self.timestep
            timestamps.append(t)

            # Prismatic Joints (X, Y, Z)
            pos_x = self.amplitude_translation * sin(2 * pi * self.frequency * t)
            pos_y = self.amplitude_translation * cos(2 * pi * self.frequency * t)
            pos_z = self.amplitude_translation * sin(2 * pi * self.frequency * t)

            vel_x = 2 * pi * self.frequency * self.amplitude_translation * cos(2 * pi * self.frequency * t)
            vel_y = -2 * pi * self.frequency * self.amplitude_translation * sin(2 * pi * self.frequency * t)
            vel_z = 2 * pi * self.frequency * self.amplitude_translation * cos(2 * pi * self.frequency * t)

            # Spherical Joint (Roll, Pitch, Yaw)
            roll = self.amplitude_rotation * sin(2 * pi * self.frequency * t)
            pitch = self.amplitude_rotation * cos(2 * pi * self.frequency * t)
            yaw = self.amplitude_rotation * sin(2 * pi * self.frequency * t / 2)

            roll_vel = 2 * pi * self.frequency * self.amplitude_rotation * cos(2 * pi * self.frequency * t)
            pitch_vel = -2 * pi * self.frequency * self.amplitude_rotation * sin(2 * pi * self.frequency * t)
            yaw_vel = pi * self.frequency * self.amplitude_rotation * cos(pi * self.frequency * t)

            joint_positions.append([pos_x, pos_y, pos_z, roll, pitch, yaw])
            joint_velocities.append([vel_x, vel_y, vel_z, roll_vel, pitch_vel, yaw_vel])

        return joint_positions, joint_velocities, timestamps

if __name__ == "__main__":
    generator = TrajectoryGenerator()
    positions, velocities, timestamps = generator.generate_trajectory()
    
    
    print(f"Time: {timestamps}s | Position: {positions} | Velocity: {velocities}")
