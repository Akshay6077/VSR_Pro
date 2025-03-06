import sys
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QCheckBox, QLineEdit, QHBoxLayout
)
from PyQt5.QtCore import Qt
import pybullet as p
from simulation_manager import SimulationManager

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
            "amplitude_rotation": 0.1  
        }

        self.init_ui()

    def init_ui(self):
        """Sets up the GUI layout and widgets."""
        self.setWindowTitle("PyBullet Simulation Control")
        self.setGeometry(100, 100, 500, 500)

        layout = QVBoxLayout()

        self.start_button = QPushButton("Start Simulation", self)
        self.start_button.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.start_button)

        # Removed Gravity Button and Simulation Speed controls

        # Add Real-Time Checkbox for trajectory execution mode
        self.realtime_checkbox = QCheckBox("Real-Time Trajectory", self)
        self.realtime_checkbox.setChecked(False)
        layout.addWidget(self.realtime_checkbox)

        self.reset_button = QPushButton("Reset Simulation", self)
        self.reset_button.clicked.connect(self.reset_simulation)
        layout.addWidget(self.reset_button)

        self.trajectory_button = QPushButton("Execute Trajectory", self)
        self.trajectory_button.clicked.connect(self.toggle_trajectory)
        layout.addWidget(self.trajectory_button)

        self.add_trajectory_controls(layout)

        self.position_label = QLabel("Pedestal Position: (0.0, 0.0, 0.0)", self)
        layout.addWidget(self.position_label)

        self.setLayout(layout)

        self.start_position_monitor()

    def start_position_monitor(self):
        """Continuously updates the pedestal position in the GUI."""
        def update_position():
            while True:
                if self.simulation_manager and self.simulation_manager.robot_id is not None:
                    try:
                        pedestal_pos, _ = p.getBasePositionAndOrientation(self.simulation_manager.robot_id)
                        self.position_label.setText(f"Pedestal Position: {pedestal_pos}")
                    except p.error:
                        print("Warning: Could not get pedestal position.")
                time.sleep(0.5)

        threading.Thread(target=update_position, daemon=True).start()

    def add_trajectory_controls(self, layout):
        """Adds trajectory parameter input fields to the layout."""
        param_names = {
            "duration": "Duration (s)",
            "timestep": "Timestep (s)",
            "frequency": "Frequency (Hz)",
            "amplitude_translation": "Translation Amplitude",
            "amplitude_rotation": "Rotation Amplitude"
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
        """Reads updated values from the GUI inputs and updates the trajectory parameters."""
        for param in self.trajectory_params:
            try:
                self.trajectory_params[param] = float(self.param_inputs[param].text())
            except ValueError:
                print(f"Warning: Invalid input for {param}, using default value.")

    def toggle_simulation(self):
        """Starts or stops the simulation."""
        if not self.running:
            self.start_simulation()
        else:
            self.stop_simulation()

    def start_simulation(self):
        """Starts the simulation in a separate thread."""
        if self.simulation_manager is None:
            self.simulation_manager = SimulationManager()
            self.simulation_manager.create_robot()

        if self.simulation_manager.robot_id is None:
            print("Error: Failed to create robot.")
            return

        self.running = True
        self.start_button.setText("Stop Simulation")

        self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stops the simulation."""
        self.running = False
        self.start_button.setText("Start Simulation")

    def run_simulation(self):
        """Runs the PyBullet simulation loop."""
        while self.running:
            if self.simulation_manager is not None:
                p.stepSimulation()
                time.sleep(0.01)
                
    def reset_simulation(self):
        """Resets the simulation and re-applies settings."""
        if self.simulation_manager:
            self.running = False
            p.resetSimulation()
            self.simulation_manager.create_robot()
            self.start_simulation()  # Restart simulation after reset

    def toggle_trajectory(self):
        """Starts or stops the trajectory execution."""
        if not self.trajectory_running:
            self.start_trajectory()
        else:
            self.stop_trajectory()

    def start_trajectory(self):
        """Starts the trajectory execution in a separate thread."""
        if self.simulation_manager is None or self.simulation_manager.robot_id is None:
            print("Error: Simulation is not running.")
            return

        # Read updated trajectory parameters from GUI inputs
        self.update_trajectory_params()

        self.trajectory_running = True
        self.trajectory_button.setText("Stop Trajectory")

        self.trajectory_thread = threading.Thread(target=self.run_trajectory, daemon=True)
        self.trajectory_thread.start()

    def stop_trajectory(self):
        """Stops the trajectory execution."""
        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

    def run_trajectory(self):
        """Runs the trajectory execution using SimulationManager's method."""
        if self.simulation_manager is None or self.simulation_manager.robot_id is None:
            print("Error: SimulationManager is not initialized.")
            return

        # Determine the mode based on the Real-Time checkbox
        real_time = self.realtime_checkbox.isChecked()
        print(f"Executing trajectory in {'real-time' if real_time else 'offline'} mode with parameters: {self.trajectory_params}")

        self.simulation_manager.execute_trajectory(self.trajectory_params, real_time=real_time)

        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = PyBulletGUI()
    gui.show()
    sys.exit(app.exec_())
