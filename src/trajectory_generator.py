# trajectory_generator.py

import math
import time
from typing import List, Tuple, Generator

class TrajectoryGenerator:
    def __init__(self, duration: float = 20.0, timestep: float = 0.01, frequency: float = 0.5, 
                 amplitude_translation: float = 0.5, amplitude_rotation: float = math.pi / 12):
        self.duration = duration
        self.timestep = timestep
        self.frequency = frequency
        self.amplitude_translation = amplitude_translation
        self.amplitude_rotation = amplitude_rotation

    def generate_trajectory_offline(self) -> Tuple[List[List[float]], List[List[float]], List[float]]:
        """Generates and returns the entire trajectory as lists for offline simulation."""
        num_steps = int(self.duration / self.timestep)
        timestamps = []
        joint_positions = []
        joint_velocities = []

        for i in range(num_steps):
            t = i * self.timestep
            timestamps.append(t)
            # Prismatic Joints (X, Y, Z)
            pos_x = self.amplitude_translation * math.sin(2 * math.pi * self.frequency * t)
            pos_y = self.amplitude_translation * math.cos(2 * math.pi * self.frequency * t)
            pos_z = self.amplitude_translation * math.sin(2 * math.pi * self.frequency * t)
            vel_x = 2 * math.pi * self.frequency * self.amplitude_translation * math.cos(2 * math.pi * self.frequency * t)
            vel_y = -2 * math.pi * self.frequency * self.amplitude_translation * math.sin(2 * math.pi * self.frequency * t)
            vel_z = 2 * math.pi * self.frequency * self.amplitude_translation * math.cos(2 * math.pi * self.frequency * t)

            # Spherical Joint (Roll, Pitch, Yaw)
            roll = self.amplitude_rotation * math.sin(2 * math.pi * self.frequency * t)
            pitch = self.amplitude_rotation * math.cos(2 * math.pi * self.frequency * t)
            yaw = self.amplitude_rotation * math.sin(2 * math.pi * self.frequency * t / 2)
            roll_vel = 2 * math.pi * self.frequency * self.amplitude_rotation * math.cos(2 * math.pi * self.frequency * t)
            pitch_vel = -2 * math.pi * self.frequency * self.amplitude_rotation * math.sin(2 * math.pi * self.frequency * t)
            yaw_vel = math.pi * self.frequency * self.amplitude_rotation * math.cos(math.pi * self.frequency * t)
            
            joint_positions.append([pos_x, pos_y, pos_z, roll, pitch, yaw])
            joint_velocities.append([vel_x, vel_y, vel_z, roll_vel, pitch_vel, yaw_vel])

        return joint_positions, joint_velocities, timestamps

    def generate_trajectory_realtime(self) -> Generator[Tuple[List[float], List[float], float], None, None]:
        """Generates the trajectory step-by-step for real-time simulation."""
        num_steps = int(self.duration / self.timestep)
        for i in range(num_steps):
            t = i * self.timestep
            # Prismatic Joints (X, Y, Z)
            pos_x = self.amplitude_translation * math.sin(2 * math.pi * self.frequency * t)
            pos_y = self.amplitude_translation * math.cos(2 * math.pi * self.frequency * t)
            pos_z = self.amplitude_translation * math.sin(2 * math.pi * self.frequency * t)
            vel_x = 2 * math.pi * self.frequency * self.amplitude_translation * math.cos(2 * math.pi * self.frequency * t)
            vel_y = -2 * math.pi * self.frequency * self.amplitude_translation * math.sin(2 * math.pi * self.frequency * t)
            vel_z = 2 * math.pi * self.frequency * self.amplitude_translation * math.cos(2 * math.pi * self.frequency * t)

            # Spherical Joint (Roll, Pitch, Yaw)
            roll = self.amplitude_rotation * math.sin(2 * math.pi * self.frequency * t)
            pitch = self.amplitude_rotation * math.cos(2 * math.pi * self.frequency * t)
            yaw = self.amplitude_rotation * math.sin(2 * math.pi * self.frequency * t / 2)
            roll_vel = 2 * math.pi * self.frequency * self.amplitude_rotation * math.cos(2 * math.pi * self.frequency * t)
            pitch_vel = -2 * math.pi * self.frequency * self.amplitude_rotation * math.sin(2 * math.pi * self.frequency * t)
            yaw_vel = math.pi * self.frequency * self.amplitude_rotation * math.cos(math.pi * self.frequency * t)

            yield [pos_x, pos_y, pos_z, roll, pitch, yaw], [vel_x, vel_y, vel_z, roll_vel, pitch_vel, yaw_vel], t
            time.sleep(self.timestep)
