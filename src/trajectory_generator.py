from typing import List, Tuple
from math import pi, sin, cos


class TrajectoryGenerator:
    """
    A class to generate trajectories for a robot with 3 prismatic joints and 1 spherical joint.

    This generator creates sinusoidal trajectories for prismatic joints (X, Y, Z) and rotational
    trajectories for a spherical joint (Roll, Pitch, Yaw), based on user-defined parameters.

    Attributes:
        num_joints (int): Number of joints in the robot (fixed to 4: 3 prismatic + 1 spherical).
    """

    def __init__(self):
        """
        Initializes the TrajectoryGenerator with a fixed number of joints.
        """
        self.num_joints = 4  # 3 prismatic joints + 1 spherical joint

    def generate_prismatic_spherical_trajectory(
        self,
        duration: float,
        timestep: float,
        frequency: float,
        amplitude_translation: float,
        rotation_amplitude: float,
    ) -> Tuple[List[List[float]], List[List[float]], List[float]]:
        """
        Generates a trajectory for a robot with 3 prismatic joints and 1 spherical joint.

        Args:
            duration (float): Total duration of the trajectory in seconds.
            timestep (float): Time step for each trajectory point in seconds.
            frequency (float): Frequency of sinusoidal motion in Hz.
            amplitude_translation (float): Amplitude of translation in meters for prismatic joints.
            rotation_amplitude (float): Amplitude of rotation in radians for the spherical joint.

        Returns:
            Tuple[List[List[float]], List[List[float]], List[float]]:
                - List of joint positions for each time step.
                  Each position contains 6 values: [X, Y, Z, Roll, Pitch, Yaw].
                - List of joint velocities for each time step.
                  Each velocity contains 6 values corresponding to the positions.
                - List of timestamps for each time step.
        """
        joint_positions = []
        joint_velocities = []
        timestamps = []

        num_steps = int(duration / timestep)
        for step in range(num_steps):
            t = step * timestep

            # Prismatic Joints (X, Y, Z)
            x = amplitude_translation * sin(2 * pi * frequency * t)
            y = amplitude_translation * cos(2 * pi * frequency * t)
            z = amplitude_translation * sin(2 * pi * frequency * t)

            # Velocities for Prismatic Joints
            vx = 2 * pi * frequency * amplitude_translation * cos(2 * pi * frequency * t)
            vy = -2 * pi * frequency * amplitude_translation * sin(2 * pi * frequency * t)
            vz = vx  # Same as X since sin(2πft) is used

            # Spherical Joint (Roll, Pitch, Yaw)
            roll = rotation_amplitude * sin(2 * pi * frequency * t)
            pitch = rotation_amplitude * cos(2 * pi * frequency * t)
            yaw = rotation_amplitude * sin(2 * pi * frequency * t / 2)

            # Velocities for Spherical Joint
            v_roll = 2 * pi * frequency * rotation_amplitude * cos(2 * pi * frequency * t)
            v_pitch = -2 * pi * frequency * rotation_amplitude * sin(2 * pi * frequency * t)
            v_yaw = pi * frequency * rotation_amplitude * cos(2 * pi * frequency * t / 2)

            # Combine positions and velocities
            joint_positions.append([x, y, z, roll, pitch, yaw])
            joint_velocities.append([vx, vy, vz, v_roll, v_pitch, v_yaw])
            timestamps.append(t)

        return joint_positions, joint_velocities, timestamps

    def validate_trajectory(
        self, joint_positions: List[List[float]], joint_velocities: List[List[float]]
    ) -> bool:
        """
        Validates the trajectory based on velocity and acceleration constraints.

        Args:
            joint_positions (list[list[float]]): Computed joint positions for the trajectory.
            joint_velocities (list[list[float]]): Computed joint velocities for the trajectory.

        Returns:
            bool: True if the trajectory is valid, False otherwise.
        """
        # Placeholder for validation logic (can include velocity/acceleration constraints)
        return True  # Assume valid for now

    def generate_waypoint_based_trajectory(
        self, waypoints: List[List[float]], time_interval: float
    ) -> Tuple[List[List[float]], List[List[float]], List[float]]:
        """
        Generates a trajectory based on user-defined waypoints.

        Args:
            waypoints (list[list[float]]): List of joint positions at each waypoint.
            time_interval (float): Time difference between consecutive waypoints.

        Returns:
            tuple: A tuple containing:
                - joint_positions (list[list[float]]): Interpolated joint positions for the trajectory.
                - joint_velocities (list[list[float]]): Interpolated joint velocities for the trajectory.
                - timestamps (list[float]): Time values corresponding to each trajectory point.
        """
        joint_positions = waypoints
        joint_velocities = []  # Compute velocities based on differences between waypoints
        timestamps = [i * time_interval for i in range(len(waypoints))]

        # Placeholder for actual interpolation logic
        return joint_positions, joint_velocities, timestamps
