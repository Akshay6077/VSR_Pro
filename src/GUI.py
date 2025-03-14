# main_gui.py

import sys
import time
import math
import threading
import pybullet as p

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, 
    QCheckBox, QLineEdit, QHBoxLayout, QFileDialog, QDialog
)
from PyQt5.QtCore import Qt

from simulation_manager import SimulationManager

###############################################################################
# Dialog to specify position/orientation
###############################################################################
class PBRLoadDialog(QDialog):
    """
    Dialog to specify position and orientation for the OBJ 
    that will be placed on the pedestal.
    """
    def __init__(self, obj_path, parent=None):
        super().__init__(parent)
        self.obj_path = obj_path
        self.setWindowTitle("Load PBR - Position & Orientation")
        
        # Defaults
        self.default_x = 0.0
        self.default_y = 0.0
        self.default_z = 0.0
        self.default_roll = 0.0
        self.default_pitch = 0.0
        self.default_yaw = 0.0

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Position row
        pos_layout = QHBoxLayout()
        pos_layout.addWidget(QLabel("X:"))
        self.x_input = QLineEdit(str(self.default_x))
        pos_layout.addWidget(self.x_input)

        pos_layout.addWidget(QLabel("Y:"))
        self.y_input = QLineEdit(str(self.default_y))
        pos_layout.addWidget(self.y_input)

        pos_layout.addWidget(QLabel("Z:"))
        self.z_input = QLineEdit(str(self.default_z))
        pos_layout.addWidget(self.z_input)

        main_layout.addLayout(pos_layout)

        # Orientation row (in degrees)
        orient_layout = QHBoxLayout()
        orient_layout.addWidget(QLabel("Roll (deg):"))
        self.roll_input = QLineEdit(str(math.degrees(self.default_roll)))
        orient_layout.addWidget(self.roll_input)

        orient_layout.addWidget(QLabel("Pitch (deg):"))
        self.pitch_input = QLineEdit(str(math.degrees(self.default_pitch)))
        orient_layout.addWidget(self.pitch_input)

        orient_layout.addWidget(QLabel("Yaw (deg):"))
        self.yaw_input = QLineEdit(str(math.degrees(self.default_yaw)))
        orient_layout.addWidget(self.yaw_input)

        main_layout.addLayout(orient_layout)

        # Confirm
        confirm_button = QPushButton("Confirm")
        confirm_button.clicked.connect(self.accept)  # sets QDialog.Accepted
        main_layout.addWidget(confirm_button)

        self.setLayout(main_layout)

    def get_values(self):
        """
        Returns (offset, orientation) in the form:
          offset = [x, y, z]
          orientation_euler = [roll_rad, pitch_rad, yaw_rad]
        """
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())

            roll_deg = float(self.roll_input.text())
            pitch_deg = float(self.pitch_input.text())
            yaw_deg = float(self.yaw_input.text())

            # Convert from degrees to radians
            roll_rad = math.radians(roll_deg)
            pitch_rad = math.radians(pitch_deg)
            yaw_rad = math.radians(yaw_deg)

        except ValueError:
            # fallback if inputs are invalid
            x, y, z = 0.0, 0.0, 0.0
            roll_rad = pitch_rad = yaw_rad = 0.0

        return [x, y, z], [roll_rad, pitch_rad, yaw_rad]


###############################################################################
# Main GUI
###############################################################################
class PyBulletGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.simulation_manager = None
        self.simulation_thread = None
        self.running = False
        self.trajectory_running = False
        self.trajectory_thread = None

        self.trajectory_params = {
            "duration": 20.0,
            "timestep": 0.01,
            "frequency": 0.5,
            "amplitude_translation": 0.5,
            "amplitude_rotation": 0.1,
        }

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("PyBullet Simulation Control")
        self.setGeometry(100, 100, 500, 500)

        layout = QVBoxLayout()

        # Start/Stop
        self.start_button = QPushButton("Start Simulation", self)
        self.start_button.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.start_button)

        # Real-Time checkbox
        self.realtime_checkbox = QCheckBox("Real-Time Trajectory", self)
        self.realtime_checkbox.setChecked(False)
        layout.addWidget(self.realtime_checkbox)

        # Reset
        self.reset_button = QPushButton("Reset Simulation", self)
        self.reset_button.clicked.connect(self.reset_simulation)
        layout.addWidget(self.reset_button)

        # Execute Trajectory
        self.trajectory_button = QPushButton("Execute Trajectory", self)
        self.trajectory_button.clicked.connect(self.toggle_trajectory)
        layout.addWidget(self.trajectory_button)

        # Upload OBJ
        self.upload_obj_button = QPushButton("Upload OBJ", self)
        self.upload_obj_button.clicked.connect(self.upload_obj)
        layout.addWidget(self.upload_obj_button)

        # Add trajectory fields
        self.add_trajectory_controls(layout)

        # Label for pedestal position
        self.position_label = QLabel("Pedestal Position: (0.0, 0.0, 0.0)", self)
        layout.addWidget(self.position_label)

        self.setLayout(layout)

        # Continuously update position label
        self.start_position_monitor()

    def add_trajectory_controls(self, layout):
        """Adds trajectory parameter input fields."""
        param_names = {
            "duration": "Duration (s)",
            "timestep": "Timestep (s)",
            "frequency": "Frequency (Hz)",
            "amplitude_translation": "Translation Amplitude",
            "amplitude_rotation": "Rotation Amplitude",
        }

        self.param_inputs = {}
        for param, label in param_names.items():
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            input_field = QLineEdit(str(self.trajectory_params[param]))
            self.param_inputs[param] = input_field
            row.addWidget(input_field)
            layout.addLayout(row)

    def update_trajectory_params(self):
        """Reads updated values from GUI and updates trajectory_params."""
        for param in self.trajectory_params:
            try:
                self.trajectory_params[param] = float(self.param_inputs[param].text())
            except ValueError:
                print(f"Warning: invalid input for {param}, using default value.")

    def start_position_monitor(self):
        """Continuously updates pedestal position label."""
        def update_position():
            while True:
                if self.simulation_manager and self.simulation_manager.robot_id is not None:
                    try:
                        # get the link state of pedestal (index=3)
                        link_state = p.getLinkState(self.simulation_manager.robot_id, 3)
                        if link_state is not None:
                            pedestal_pos = link_state[0]
                            self.position_label.setText(f"Pedestal Position: {pedestal_pos}")
                    except p.error:
                        pass
                time.sleep(0.5)

        threading.Thread(target=update_position, daemon=True).start()

    ###########################################################################
    # Simulation
    ###########################################################################
    def toggle_simulation(self):
        """Start or stop the simulation."""
        if not self.running:
            self.start_simulation()
        else:
            self.stop_simulation()

    def start_simulation(self):
        """Starts the PyBullet simulation in a separate thread."""
        if self.simulation_manager is None:
            # Path to your YAML config file
            self.simulation_manager = SimulationManager("config/6dof_vsr.yaml")
            self.simulation_manager.create_robot()

        if self.simulation_manager.robot_id is None:
            print("Error: Failed to create robot.")
            return

        self.running = True
        self.start_button.setText("Stop Simulation")

        self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.simulation_thread.start()

    def stop_simulation(self):
        self.running = False
        self.start_button.setText("Start Simulation")

    def run_simulation(self):
        """Main simulation loop."""
        while self.running:
            if self.simulation_manager is not None:
                p.stepSimulation()
            time.sleep(0.01)

    def reset_simulation(self):
        """Resets the simulation and re-creates the robot."""
        if self.simulation_manager:
            self.running = False
            p.resetSimulation()
            self.simulation_manager.create_robot()
            self.start_simulation()

    ###########################################################################
    # Trajectory
    ###########################################################################
    def toggle_trajectory(self):
        if not self.trajectory_running:
            self.start_trajectory()
        else:
            self.stop_trajectory()

    def start_trajectory(self):
        if self.simulation_manager is None or self.simulation_manager.robot_id is None:
            print("Error: Simulation is not running.")
            return

        self.update_trajectory_params()
        self.trajectory_running = True
        self.trajectory_button.setText("Stop Trajectory")

        self.trajectory_thread = threading.Thread(target=self.run_trajectory, daemon=True)
        self.trajectory_thread.start()

    def stop_trajectory(self):
        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

    def run_trajectory(self):
        if self.simulation_manager is None or self.simulation_manager.robot_id is None:
            print("Error: SimulationManager is not initialized.")
            return

        real_time = self.realtime_checkbox.isChecked()
        print(f"Executing trajectory in {'real-time' if real_time else 'offline'} mode with {self.trajectory_params}")
        self.simulation_manager.execute_trajectory(self.trajectory_params, real_time=real_time)

        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

    ###########################################################################
    # OBJ Upload
    ###########################################################################
    def upload_obj(self):
        """Select an OBJ file and spawn it with user-defined position/orientation."""
        if self.simulation_manager is None or self.simulation_manager.robot_id is None:
            print("Error: Simulation must be running to upload an OBJ.")
            return

        obj_path, _ = QFileDialog.getOpenFileName(self, "Select OBJ/PBR File", "", "OBJ Files (*.obj)")
        if obj_path:
            print(f"Selected OBJ file: {obj_path}")

            # Popup dialog to specify offset/orientation
            dialog = PBRLoadDialog(obj_path, parent=self)
            if dialog.exec_() == QDialog.Accepted:
                offset, orientation_euler = dialog.get_values()
                self.simulation_manager.spawn_obj_on_pedestal(
                    obj_path,
                    mesh_scale=[1, 1, 1],
                    offset=offset,
                    orientation_euler=orientation_euler
                )

###############################################################################
# Main Entry Point
###############################################################################
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = PyBulletGUI()
    gui.show()
    sys.exit(app.exec_())
