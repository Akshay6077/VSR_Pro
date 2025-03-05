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

         # ✅ Increase Solver Accuracy
        p.setPhysicsEngineParameter(
            numSolverIterations=1000,  # High iterations for better contact resolution
            numSubSteps=10,            # More substeps improve physics accuracy
            physicsClientId=self.client_id
        )
        # Load structure configurations
        self.world_box_config = self.config["structure"]["world_box"]
        self.pedestal_config = self.config["structure"]["pedestal"]
        self.dynamics_config = self.config["dynamics"]
        self.joint_config = self.config["joints"]
        self.robot_visual_config = self.config["robot_visual"]

        self.robot_id = None

    def create_robot(self):
        """Creates the 6-DOF robot with prismatic and spherical joints, ensuring stability."""

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

        world_box_position = [0, 0, world_box_half_extents[2]]  # ✅ Correct Ground Position

        # ✅ Step 2: Define Pedestal (Moving Base) with Gap
        pedestal_half_extents = [dim / 2 for dim in self.pedestal_config["dimensions"]]
        pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=pedestal_half_extents)
        pedestal_visual_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=pedestal_half_extents, rgbaColor=self.robot_visual_config["pedestal_color"]
        )

        pedestal_gap = 0.2  # ✅ Small gap for tolerance

        pedestal_position = [
            0, 0, 
            world_box_position[2] + world_box_half_extents[2] + pedestal_half_extents[2] + pedestal_gap  # ✅ Correct Height
        ]

        print(f"🔍 World Box Position: {world_box_position}")
        print(f"🔍 Pedestal Expected Position: {pedestal_position}")

        # ✅ Step 3: Define Joints (3 Prismatic + 1 Spherical)
        num_links = 4  # ✅ Must match in all arrays

        link_masses = [self.pedestal_config["mass"]] * num_links
        link_collision_shapes = [-1, -1, -1, pedestal_collision_shape]  # ✅ Ensure same length
        link_visual_shapes = [-1, -1, -1, pedestal_visual_shape]  # ✅ Ensure same length
        link_positions = [
            [0, 0, world_box_half_extents[2]],  # X Prismatic starts on top of the base
            [0, 0, 0.2],  # Y Prismatic
            [0, 0, 0.3],  # Z Prismatic
            pedestal_position  # ✅ Pedestal placed above base
        ]
        link_orientations = [[0, 0, 0, 1]] * num_links
        link_inertial_positions = [[0, 0, 0]] * num_links
        link_inertial_orientations = [[0, 0, 0, 1]] * num_links
        link_parent_indices = [0, 1, 2, 3]  # ✅ Must match num_links
        link_joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
        link_joint_axes = [
            self.joint_config["prismatic_x"]["axis"],
            self.joint_config["prismatic_y"]["axis"],
            self.joint_config["prismatic_z"]["axis"],
            self.joint_config["spherical_joint"]["axis"]
        ]

        # ✅ Ensure that all arrays have the same number of elements
        assert len(link_masses) == num_links
        assert len(link_collision_shapes) == num_links
        assert len(link_visual_shapes) == num_links
        assert len(link_positions) == num_links
        assert len(link_orientations) == num_links
        assert len(link_inertial_positions) == num_links
        assert len(link_inertial_orientations) == num_links
        assert len(link_parent_indices) == num_links
        assert len(link_joint_types) == num_links
        assert len(link_joint_axes) == num_links

        # ✅ Create the MultiBody with Properly Matched Arrays
        self.robot_id = p.createMultiBody(
            baseMass=0.0,  # ✅ Small non-zero mass for collision response
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
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes
        )

        if self.robot_id < 0:
            raise RuntimeError("Failed to create robot!")

        print(f"✅ Robot created with ID: {self.robot_id}")

        # ✅ Enable Collision Between Base & Pedestal
        p.setCollisionFilterPair(self.robot_id, self.robot_id, -1, 3, enableCollision=1)
        p.changeDynamics(
        self.robot_id, 2,  # Z-prismatic joint
        jointLowerLimit=pedestal_gap,  # ✅ Minimum allowed height
        jointUpperLimit=3.0,  # ✅ Allow some movement
        maxJointVelocity=0.1,  # ✅ Prevent sudden downward motion
        physicsClientId=self.client_id
        )


        # ✅ Apply Dynamics Separately
        self.apply_dynamics()

        print("🚀 Pedestal should now stay above the base with a small gap and move correctly.")


    def apply_dynamics(self):
        """Applies and prints dynamics properties from YAML for verification."""

        print("Applying Dynamics Properties...")

        # ✅ Apply World Box Dynamics
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

        print(f"✅ World Box Dynamics Applied: {p.getDynamicsInfo(self.robot_id, -1)}")

        # ✅ Apply Pedestal Dynamics
        for i in range(4):  # Apply to all links (joints)
            p.changeDynamics(
                self.robot_id,
                i,
                restitution=self.dynamics_config["pedestal"]["restitution"],
                lateralFriction=self.dynamics_config["pedestal"]["lateralFriction"],
                spinningFriction=self.dynamics_config["pedestal"]["spinningFriction"],
                contactDamping=self.dynamics_config["pedestal"]["contactDamping"],
                contactStiffness=self.dynamics_config["pedestal"]["contactStiffness"],
                localInertiaDiagonal=self.pedestal_config["inertia"],  # ✅ Apply Inertia from YAML
                physicsClientId=self.client_id
            )

            print(f"✅ Pedestal Link {i} Dynamics: {p.getDynamicsInfo(self.robot_id, i)}")
            
    def execute_trajectory(self, trajectory_params):
        """Executes a trajectory using user-updated parameters from GUI."""

        if self.robot_id is None:
            print("Error: Robot has not been created yet.")
            return

        print(f"Executing trajectory with parameters: {trajectory_params}")

        # ✅ Create a new trajectory using updated user inputs
        generator = TrajectoryGenerator(
            duration=trajectory_params["duration"],
            timestep=trajectory_params["timestep"],
            frequency=trajectory_params["frequency"],
            amplitude_translation=trajectory_params["amplitude_translation"],
            amplitude_rotation=trajectory_params["amplitude_rotation"]
        )

        joint_positions, joint_velocities, timestamps = generator.generate_trajectory()

        print("Executing trajectory...")

        for i in range(len(timestamps)):
            # ✅ Control prismatic joints (X, Y, Z)
            p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL, targetPosition=joint_positions[i][0])
            p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL, targetPosition=joint_positions[i][1])
            p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL, targetPosition=joint_positions[i][2])

            # ✅ Control spherical joint (Roll, Pitch, Yaw)
            target_orientation = p.getQuaternionFromEuler([
                joint_positions[i][3],  # Roll
                joint_positions[i][4],  # Pitch
                joint_positions[i][5]   # Yaw
            ])
            p.setJointMotorControlMultiDof(self.robot_id, 3, p.POSITION_CONTROL, targetPosition=target_orientation)

            p.stepSimulation()
            time.sleep(trajectory_params["timestep"])

        print("Trajectory execution complete.")



if __name__ == "__main__":
    sim_manager = SimulationManager()
    sim_manager.create_robot()
    sim_manager.execute_trajectory()

      # Keep the simulation running to observe the pedestal behavior
    while True:
        p.stepSimulation()
        time.sleep(0.01)