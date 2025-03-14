# simulation_manager.py

import yaml
import pybullet as p
import pybullet_data
import time

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
        p.setRealTimeSimulation(1 if self.use_real_time else 0)

        # Increase Solver Accuracy
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
        """Creates the 6-DOF robot with prismatic and spherical joints."""
        print("Creating 6-DOF Robot...")

        # Load the Ground Plane
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        print(f"✅ Plane loaded with ID: {plane_id}")

        # Step 1: Create World Box (Fixed Base)
        world_box_half_extents = [dim / 2 for dim in self.world_box_config["dimensions"]]
        world_box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=world_box_half_extents)
        world_box_visual_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=world_box_half_extents, rgbaColor=self.robot_visual_config["world_box_color"]
        )
        world_box_position = [0, 0, world_box_half_extents[2]]  # Correct Ground Position

        # Step 2: Define Pedestal (Moving Base) with Gap
        pedestal_half_extents = [dim / 2 for dim in self.pedestal_config["dimensions"]]
        pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=pedestal_half_extents)
        pedestal_visual_shape = p.createVisualShape(
            p.GEOM_BOX, halfExtents=pedestal_half_extents, rgbaColor=self.robot_visual_config["pedestal_color"]
        )
        pedestal_gap = 0.0  # small gap

        pedestal_position = [
            0, 
            0, 
            world_box_position[2] + world_box_half_extents[2] + pedestal_half_extents[2] + pedestal_gap
        ]
        print(f"🔍 World Box Position: {world_box_position}")
        print(f"🔍 Pedestal Expected Position: {pedestal_position}")

        # Step 3: Define Joints (3 Prismatic + 1 Spherical)
        num_links = 4
        link_masses = [self.pedestal_config["mass"]] * num_links
        link_collision_shapes = [-1, -1, -1, pedestal_collision_shape]
        link_visual_shapes = [-1, -1, -1, pedestal_visual_shape]
        link_positions = [
            [0, 0, world_box_half_extents[2]],
            [0, 0, 0.2],
            [0, 0, 0.3],
            pedestal_position
        ]
        link_orientations = [[0, 0, 0, 1]] * num_links
        link_inertial_positions = [[0, 0, 0]] * num_links
        link_inertial_orientations = [[0, 0, 0, 1]] * num_links
        link_parent_indices = [0, 1, 2, 3]
        link_joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
        link_joint_axes = [
            self.joint_config["prismatic_x"]["axis"],
            self.joint_config["prismatic_y"]["axis"],
            self.joint_config["prismatic_z"]["axis"],
            self.joint_config["spherical_joint"]["axis"]
        ]

        self.robot_id = p.createMultiBody(
            baseMass=0.0,
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

        # Enable collision between base & pedestal
        p.setCollisionFilterPair(self.robot_id, self.robot_id, -1, 3, enableCollision=1)

        # Set some constraints on the Z prismatic joint
        p.changeDynamics(
            self.robot_id, 2,
            jointLowerLimit=pedestal_gap,
            jointUpperLimit=3.0,
            maxJointVelocity=0.1,
            physicsClientId=self.client_id
        )

        # Apply dynamics
        self.apply_dynamics()
        print("🚀 Pedestal created and dynamics applied.")

    def apply_dynamics(self):
        """Applies and prints dynamics properties from YAML."""
        print("Applying Dynamics Properties...")

        # World Box base
        p.changeDynamics(
            self.robot_id,
            -1,
            restitution=self.dynamics_config["world_box"]["restitution"],
            lateralFriction=self.dynamics_config["world_box"]["lateralFriction"],
            spinningFriction=self.dynamics_config["world_box"]["spinningFriction"],
            contactDamping=self.dynamics_config["world_box"]["contactDamping"],
            contactStiffness=self.dynamics_config["world_box"]["contactStiffness"],
            physicsClientId=self.client_id
        )
        print(f"✅ World Box Dynamics: {p.getDynamicsInfo(self.robot_id, -1)}")

        # Pedestal (links 0,1,2,3)
        for i in range(4):
            p.changeDynamics(
                self.robot_id,
                i,
                restitution=self.dynamics_config["pedestal"]["restitution"],
                lateralFriction=self.dynamics_config["pedestal"]["lateralFriction"],
                spinningFriction=self.dynamics_config["pedestal"]["spinningFriction"],
                contactDamping=self.dynamics_config["pedestal"]["contactDamping"],
                contactStiffness=self.dynamics_config["pedestal"]["contactStiffness"],
                localInertiaDiagonal=self.pedestal_config["inertia"],
                physicsClientId=self.client_id
            )
            print(f"✅ Pedestal Link {i} Dynamics: {p.getDynamicsInfo(self.robot_id, i)}")

    def execute_trajectory(self, trajectory_params, real_time: bool = False):
        """Executes a trajectory using user-updated parameters."""
        if self.robot_id is None:
            print("Error: Robot has not been created yet.")
            return

        print(f"Executing trajectory with parameters: {trajectory_params}")
        generator = TrajectoryGenerator(
            duration=trajectory_params["duration"],
            timestep=trajectory_params["timestep"],
            frequency=trajectory_params["frequency"],
            amplitude_translation=trajectory_params["amplitude_translation"],
            amplitude_rotation=trajectory_params["amplitude_rotation"]
        )

        if real_time:
            for joint_pos, joint_vel, t in generator.generate_trajectory_realtime():
                # Indices 0,1,2 => prismatic; index 3 => spherical
                p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL, targetPosition=joint_pos[0])
                p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL, targetPosition=joint_pos[1])
                p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL, targetPosition=joint_pos[2])

                target_orientation = p.getQuaternionFromEuler(joint_pos[3:6])
                p.setJointMotorControlMultiDof(self.robot_id, 3, p.POSITION_CONTROL, 
                                               targetPosition=target_orientation)
                p.stepSimulation()
        else:
            offline_speed_factor = 0.5  # run faster than real-time
            print("Executing trajectory offline... Waiting 2 seconds before starting...")
            time.sleep(2)

            joint_positions, joint_velocities, timestamps = generator.generate_trajectory_offline()
            for i in range(len(timestamps)):
                p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL, targetPosition=joint_positions[i][0])
                p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL, targetPosition=joint_positions[i][1])
                p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL, targetPosition=joint_positions[i][2])

                target_orientation = p.getQuaternionFromEuler(joint_positions[i][3:6])
                p.setJointMotorControlMultiDof(self.robot_id, 3, p.POSITION_CONTROL, 
                                               targetPosition=target_orientation)
                p.stepSimulation()
                time.sleep(trajectory_params["timestep"] * offline_speed_factor)

        print("Trajectory execution complete.")

    def spawn_obj_on_pedestal(self, 
                              obj_path: str, 
                              mesh_scale=[1, 1, 1],
                              offset=[0.0, 0.0, 0.0],
                              orientation_euler=[0.0, 0.0, 0.0]):
        """
        Spawns an OBJ file on top of the pedestal with a user-defined offset
        and orientation (Euler angles in radians).

        :param obj_path: Path to the .obj file
        :param mesh_scale: [sx, sy, sz]
        :param offset: [dx, dy, dz]
        :param orientation_euler: [roll, pitch, yaw] in radians
        """
        if self.robot_id is None:
            print("Error: Robot has not been created yet.")
            return

        # Get link index 3 position
        pedestal_state = p.getLinkState(self.robot_id, 3)
        pedestal_position = pedestal_state[0]

        final_position = [
            pedestal_position[0] + offset[0],
            pedestal_position[1] + offset[1],
            pedestal_position[2] + offset[2],
        ]
        final_orientation = p.getQuaternionFromEuler(orientation_euler)

        obj_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            meshScale=mesh_scale
        )

        # If you need collision, uncomment
        # obj_collision_shape = p.createCollisionShape(
        #     shapeType=p.GEOM_MESH,
        #     fileName=obj_path,
        #     meshScale=mesh_scale
        # )

        obj_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=obj_visual_shape,
            basePosition=final_position,
            baseOrientation=final_orientation,
            # baseCollisionShapeIndex=obj_collision_shape,
        )
        print(
            f"Spawned OBJ from '{obj_path}' at position {final_position}, "
            f"orientation={orientation_euler}, ID: {obj_id}"
        )
        return obj_id
