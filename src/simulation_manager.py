import pybullet as p
import pybullet_data
import yaml
import time
import os

class SimulationManager:
    """
    A class to manage the PyBullet simulation for a 6-DOF robot.

    Responsibilities:
    - Initialize and configure the PyBullet physics engine.
    - Load simulation settings from a YAML configuration file.
    - Create the robot with prismatic and spherical joints.
    - Execute provided trajectories on the robot.

    Attributes:
    ----------
    client_id : int
        ID of the PyBullet client connection.
    robot_id : int
        ID of the robot in the simulation.
    """

    def __init__(self, yaml_path="/home/akshay/ros2_ws/src/VSR_Pro/src/config/6dof_vsr.yaml"):
        """
        Initializes the PyBullet simulation and loads configuration from a YAML file.

        Args:
            yaml_path (str): Path to the YAML configuration file.
        """
        # Load YAML Configuration
        with open(yaml_path, "r") as file:
            self.config = yaml.safe_load(file)

        # Extract Simulation Settings
        self.gravity = tuple(self.config["simulation_settings"]["gravity"])
        self.time_step = self.config["simulation_settings"]["time_step"]
        self.use_gui = self.config["simulation_settings"]["enable_gui"]

        # Initialize PyBullet
        self.client_id = p.connect(p.GUI if self.use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Configure Physics
        p.setTimeStep(self.time_step, self.client_id)
        p.setGravity(*self.gravity, self.client_id)

        # Extract Structure Data
        self.world_box_config = self.config["structure"]["world_box"]
        self.pedestal_config = self.config["structure"]["pedestal"]

        # Extract Joint Configurations
        self.joint_config = self.config["joints"]

        self.robot_id = None

    def create_robot(self):
        """
        Creates a 6-DOF robot using prismatic joints for X, Y, Z translation and a spherical joint for rotation.
        """
        print("Creating 6-DOF Robot...")

        # ✅ Load the Ground Plane
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        print(f"Plane loaded with ID: {plane_id}")

        # ✅ Step 1: Create World Box (Fixed Base)
        world_box_half_extents = [dim / 2 for dim in self.world_box_config["dimensions"]]
        world_box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=world_box_half_extents)
        world_box_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=world_box_half_extents, rgbaColor=self.config["robot_visual"]["world_box_color"])
        
        world_box_position = [0, 0, world_box_half_extents[2]]

        # ✅ Step 2: Create Pedestal (Moving Base)
        pedestal_half_extents = [dim / 2 for dim in self.pedestal_config["dimensions"]]
        pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=pedestal_half_extents)
        pedestal_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=pedestal_half_extents, rgbaColor=self.config["robot_visual"]["pedestal_color"])

        # Pedestal starts above the world box
        pedestal_position = [0, 0, world_box_half_extents[2] + pedestal_half_extents[2]]

        # ✅ Step 3: Define Joints (3 Prismatic + 1 Spherical)
        num_links = 4  # Three prismatic joints and one spherical joint

        link_masses = [self.pedestal_config["mass"]] * num_links
        link_collision_shapes = [-1, -1, -1, pedestal_collision_shape]  # Last link has collision shape
        link_visual_shapes = [-1, -1, -1, pedestal_visual_shape]  # Last link is visible

        link_positions = [[0, 0, pedestal_half_extents[2]],  # X Prismatic
                        [0, 0, 0.2],  # Y Prismatic
                        [0, 0, 0.3],  # Z Prismatic
                        [0, 0, 0.4]]  # Spherical (Final Link)]
        
        link_orientations = [[0, 0, 0, 1]] * num_links
        link_inertial_positions = [[0, 0, 0]] * num_links
        link_inertial_orientations = [[0, 0, 0, 1]] * num_links
        link_parent_indices = [0, 1, 2, 3]  # Each joint connects sequentially

        # Joint Types and Axes
        joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
        joint_axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]]  # X, Y, Z translation, and rotation

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

        print(f"Robot created with ID: {self.robot_id}")

        # ✅ Step 5: Configure Joint Dynamics for Stability
        for i in range(num_links):
            p.changeDynamics(
                self.robot_id,
                i,
                lateralFriction=0.8,
                spinningFriction=0.3,
                restitution=0.1,
                physicsClientId=self.client_id
            )

        print("6-DOF Robot successfully created!")

        num_joints = p.getNumJoints(sim_manager.robot_id, physicsClientId=sim_manager.client_id)
        print(f"Number of joints in the robot: {num_joints}")

    def execute_trajectory(self, joint_positions, joint_velocities, timestamps):
        """
        Executes a trajectory for the robot's joints.

        Args:
            joint_positions (list[list[float]]): Joint positions at each time step.
            joint_velocities (list[list[float]]): Joint velocities at each time step.
            timestamps (list[float]): Time values corresponding to each trajectory point.
        """
        print("Executing trajectory...")

        for i in range(len(timestamps)):
            time.sleep(self.time_step)

            # Apply trajectory to prismatic joints
            for j in range(3):  # First three joints are prismatic
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=joint_positions[i][j],
                    force=self.joint_config[f"prismatic_{['x', 'y', 'z'][j]}"]["max_force"],
                    physicsClientId=self.client_id
                )

            # Apply trajectory to spherical joint
            spherical_target_orientation = p.getQuaternionFromEuler(joint_positions[i][3:])
            p.setJointMotorControlMultiDof(
                bodyUniqueId=self.robot_id,
                jointIndex=3,
                controlMode=p.POSITION_CONTROL,
                targetPosition=spherical_target_orientation,
                physicsClientId=self.client_id
            )

            # Step simulation
            p.stepSimulation(self.client_id)

        print("Trajectory execution complete.")

    def reset_simulation(self):
        """Resets the simulation to its initial state."""
        p.resetSimulation(self.client_id)
        print("Simulation reset.")

    def disconnect(self):
        """Disconnects the PyBullet client."""
        p.disconnect(self.client_id)
        print("Simulation disconnected.")


if __name__ == "__main__":
    sim_manager = SimulationManager()

    # Load the robot
    sim_manager.create_robot()

    # Keep simulation running
    input("Press ENTER to exit...")
    sim_manager.disconnect()
