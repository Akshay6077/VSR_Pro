import sys
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QSlider, QCheckBox, QLineEdit, QHBoxLayout
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

        self.gravity_checkbox = QCheckBox("Enable Gravity", self)
        self.gravity_checkbox.setChecked(True)
        self.gravity_checkbox.stateChanged.connect(self.toggle_gravity)
        layout.addWidget(self.gravity_checkbox)

        self.speed_label = QLabel("Simulation Speed: 1x", self)
        layout.addWidget(self.speed_label)

        self.speed_slider = QSlider(Qt.Horizontal, self)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(10)
        self.speed_slider.setValue(1)
        self.speed_slider.valueChanged.connect(self.update_speed)
        layout.addWidget(self.speed_slider)

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
                
    def toggle_gravity(self):
        """Toggles gravity in the simulation and applies the change immediately."""
        if self.simulation_manager:
            if self.gravity_checkbox.isChecked():
                print("✅ Gravity Enabled")
                p.setGravity(0, 0, -9.81)
            else:
                print("🚀 Gravity Disabled")
                p.setGravity(0, 0, 0)

            # ✅ Re-apply forces after gravity change to ensure objects update
            p.stepSimulation()


    def update_speed(self):
        """Updates the simulation speed."""
        speed = self.speed_slider.value()
        self.speed_label.setText(f"Simulation Speed: {speed}x")
        if self.simulation_manager:
            p.setTimeStep(0.01 / speed)

    def reset_simulation(self):
        """Resets the simulation and re-applies gravity."""
        if self.simulation_manager:
            self.running = False
            p.resetSimulation()
            self.simulation_manager.create_robot()

            # ✅ Ensure gravity is re-applied based on the checkbox state
            if self.gravity_checkbox.isChecked():
                p.setGravity(0, 0, -9.81)
            else:
                p.setGravity(0, 0, 0)

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

        # Read updated values from GUI inputs
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

        print(f"Executing trajectory with parameters: {self.trajectory_params}")

        # Call execute_trajectory() in SimulationManager
        self.simulation_manager.execute_trajectory(self.trajectory_params)

        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = PyBulletGUI()
    gui.show()
    sys.exit(app.exec_())
