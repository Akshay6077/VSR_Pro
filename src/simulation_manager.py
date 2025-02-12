import pybullet as p
import pybullet_data
import yaml
import time
import os
from trajectory_generator import TrajectoryGenerator


class SimulationManager:
    """
    Manages the PyBullet simulation for a 6-DOF robot.
    
    Responsibilities:
    - Initialize PyBullet physics engine.
    - Load settings from a YAML file.
    - Create and control a 6-DOF robot.
    """

    def __init__(self, yaml_path="config/6dof_vsr.yaml"):
        """Initialize simulation from YAML configuration."""
        
        # Load YAML Configuration
        with open(yaml_path, "r") as file:
            self.config = yaml.safe_load(file)

        # Extract Simulation Settings
        self.gravity = tuple(self.config["simulation_settings"]["gravity"])
        self.time_step = self.config["simulation_settings"]["time_step"]
        self.use_gui = self.config["simulation_settings"]["enable_gui"]
        self.use_real_time = self.config["simulation_settings"]["use_real_time"]

        # Ensure no duplicate PyBullet connections
        if p.isConnected():
            p.disconnect()

        # Initialize PyBullet
        self.client_id = p.connect(p.GUI if self.use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set Physics Properties
        p.setTimeStep(self.time_step, self.client_id)
        p.setGravity(*self.gravity, self.client_id)
        if self.use_real_time:
            p.setRealTimeSimulation(1)
        else:
            p.setRealTimeSimulation(0)

        # Load structure configurations
        self.world_box_config = self.config["structure"]["world_box"]
        self.pedestal_config = self.config["structure"]["pedestal"]
        self.dynamics_config = self.config["dynamics"]
        self.joint_config = self.config["joints"]
        self.robot_visual_config = self.config["robot_visual"]

        self.robot_id = None

    def create_robot(self):
        """Creates the 6-DOF robot with prismatic and spherical joints."""

        print("Creating 6-DOF Robot...")
        
        # ✅ Load the Ground Plane
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        print(f"✅ Plane loaded with ID: {plane_id}")

        # ✅ Step 1: Create World Box (Fixed Base)
        world_box_half_extents = [dim / 2 for dim in self.world_box_config["dimensions"]]
        world_box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=world_box_half_extents)
        world_box_visual_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=world_box_half_extents, rgbaColor=self.robot_visual_config["world_box_color"]
        )
        world_box_position = [0, 0, world_box_half_extents[2]]

        # ✅ Step 2: Create Pedestal (Moving Base)
        pedestal_half_extents = [dim / 2 for dim in self.pedestal_config["dimensions"]]
        pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=pedestal_half_extents)
        pedestal_visual_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=pedestal_half_extents, rgbaColor=self.robot_visual_config["pedestal_color"]
        )
        pedestal_position = [0, 0, world_box_half_extents[2] + pedestal_half_extents[2]]

        # ✅ Step 3: Define Joints (3 Prismatic + 1 Spherical)
        num_links = 4  # Three prismatic joints and one spherical joint

        link_masses = [self.pedestal_config["mass"]] * num_links
        link_collision_shapes = [-1, -1, -1, pedestal_collision_shape]  # Only last link has a collision shape
        link_visual_shapes = [-1, -1, -1, pedestal_visual_shape]  # Only last link is visible

        link_positions = [[0, 0, pedestal_half_extents[2]],  # X Prismatic
                          [0, 0, 0.2],  # Y Prismatic
                          [0, 0, 0.3],  # Z Prismatic
                          [0, 0, 0.4]]  # Spherical joint

        link_orientations = [[0, 0, 0, 1]] * num_links
        link_inertial_positions = [[0, 0, 0]] * num_links
        link_inertial_orientations = [[0, 0, 0, 1]] * num_links
        link_parent_indices = [0, 1, 2, 3]  # Sequentially linked

        # Joint Types and Axes
        joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
        joint_axes = [
            self.joint_config["prismatic_x"]["axis"],
            self.joint_config["prismatic_y"]["axis"],
            self.joint_config["prismatic_z"]["axis"],
            self.joint_config["spherical_joint"]["axis"]
        ]

        # ✅ Step 4: Create MultiBody
        self.robot_id = p.createMultiBody(
            baseMass=self.world_box_config["mass"],
            baseCollisionShapeIndex=world_box_collision_shape,
            baseVisualShapeIndex=world_box_visual_shape,
            basePosition=world_box_position,
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collision_shapes,
            linkVisualShapeIndices=link_visual_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_positions,
            linkInertialFrameOrientations=link_inertial_orientations,
            linkParentIndices=link_parent_indices,
            linkJointTypes=joint_types,
            linkJointAxis=joint_axes
        )

        if self.robot_id < 0:
            raise RuntimeError("Failed to create robot!")

        print(f" Robot created with ID: {self.robot_id}")

        # Apply Dynamics
        self.apply_dynamics()

    def apply_dynamics(self):
        """Applies and prints dynamics properties from YAML for verification."""

        print("Applying Dynamics Properties...")

        # Apply World Box Dynamics
        p.changeDynamics(
            self.robot_id,
            -1,  # Base ID
            restitution=self.dynamics_config["world_box"]["restitution"],
            lateralFriction=self.dynamics_config["world_box"]["lateralFriction"],
            spinningFriction=self.dynamics_config["world_box"]["spinningFriction"],
            contactDamping=self.dynamics_config["world_box"]["contactDamping"],
            contactStiffness=self.dynamics_config["world_box"]["contactStiffness"],
            physicsClientId=self.client_id
        )
        print(f" World Box Dynamics Applied: {p.getDynamicsInfo(self.robot_id, -1)}")

        # Apply Pedestal Dynamics
        for i in range(4):  # Apply to all links (joints)
            p.changeDynamics(
                self.robot_id,
                i,
                restitution=self.dynamics_config["pedestal"]["restitution"],
                lateralFriction=self.dynamics_config["pedestal"]["lateralFriction"],
                spinningFriction=self.dynamics_config["pedestal"]["spinningFriction"],
                contactDamping=self.dynamics_config["pedestal"]["contactDamping"],
                contactStiffness=self.dynamics_config["pedestal"]["contactStiffness"],
                physicsClientId=self.client_id
            )
            print(f"Pedestal Link {i} Dynamics: {p.getDynamicsInfo(self.robot_id, i)}")

    def execute_trajectory(self):
        """Executes a trajectory using TrajectoryGenerator."""
        generator = TrajectoryGenerator()
        joint_positions, joint_velocities, timestamps = generator.generate_trajectory()

        print("Executing trajectory...")

        for i in range(len(timestamps)):
            p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL, targetPosition=joint_positions[i][0])
            p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL, targetPosition=joint_positions[i][1])
            p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL, targetPosition=joint_positions[i][2])

            target_orientation = p.getQuaternionFromEuler([joint_positions[i][3], joint_positions[i][4], joint_positions[i][5]])
            p.setJointMotorControlMultiDof(self.robot_id, 3, p.POSITION_CONTROL, targetPosition=target_orientation)

            p.stepSimulation()
            time.sleep(generator.timestep)

        print("Trajectory execution complete.")


if __name__ == "__main__":
    sim_manager = SimulationManager()
    sim_manager.create_robot()
    # sim_manager.execute_trajectory()

      # Keep the simulation running to observe the pedestal behavior
    while True:
        p.stepSimulation()
        time.sleep(0.01)
