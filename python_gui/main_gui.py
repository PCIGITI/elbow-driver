import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
from PIL import Image, ImageTk
import os

from constants import MotorIndex
from serial_comm import SerialCommunicator
from motor_control import MotorController
from test_motors_window import TestMotorsWindow
from utils import PathLengthCalculator

class ElbowSimulatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Elbow Simulator Control")
        
        # Initialize components
        self.serial_comm = SerialCommunicator(callback=self.process_serial_response)
        self.motor_controller = MotorController()
        self.path_calculator = PathLengthCalculator()
        
        # GUI state variables
        self.is_connected = False
        self.is_verbose = False
        self.selected_motor = "EPU"
        self.test_motors_window = None
        
        self.setup_variables()
        self.create_widgets()

    def setup_variables(self):
        # Step control
        self.step_sizes = [1, 5, 10, 25, 50, 100, 200]
        self.current_step_size_var = tk.StringVar(value="2")  # Index 2 = 10 steps
        
        # Joint control variables
        self.ep_degrees_input_var = tk.StringVar(value="0")
        self.ey_degrees_input_var = tk.StringVar(value="0")
        self.wp_degrees_input_var = tk.StringVar(value="0")
        self.lj_degrees_input_var = tk.StringVar(value="0")
        self.rj_degrees_input_var = tk.StringVar(value="0")
        
        # Cumulative position tracking
        self.cumulative_ep_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_ey_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_wp_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_lj_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_rj_degrees_var = tk.DoubleVar(value=0.0)

    def create_widgets(self):
        self.main_frame = ttk.Frame(self.root, padding="5")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        self._create_connection_frame()
        self._create_movement_frame()
        self._create_system_commands_frame()
        self._create_positional_control_frame()
        self._create_output_area()

    def _create_connection_frame(self):
        conn_frame = ttk.LabelFrame(self.main_frame, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        self.port_entry = ttk.Entry(conn_frame, width=15)
        self.port_entry.insert(0, "COM3")
        self.port_entry.grid(row=0, column=0, padx=5)
        
        self.connect_button = ttk.Button(conn_frame, text="Connect",
                                       command=self.toggle_connection)
        self.connect_button.grid(row=0, column=1, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected")
        self.status_label.grid(row=0, column=2, padx=5)

    def _create_movement_frame(self):
        movement_frame = ttk.LabelFrame(self.main_frame, text="Movement Control", padding="5")
        movement_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        slider_frame = ttk.Frame(movement_frame)
        slider_frame.pack(fill=tk.X)
        
        ttk.Label(slider_frame, text="Step Size:").pack(side=tk.LEFT)
        self.step_size_slider = ttk.Scale(slider_frame, from_=0,
                                        to=len(self.step_sizes)-1,
                                        orient=tk.HORIZONTAL,
                                        variable=self.current_step_size_var,
                                        command=self.update_step_size_label)
        self.step_size_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.step_size_display_label = ttk.Label(slider_frame, text="10 steps", width=10)
        self.step_size_display_label.pack(side=tk.LEFT)

    def _create_system_commands_frame(self):
        sys_cmd_frame = ttk.LabelFrame(self.main_frame, text="System Commands", padding="5")
        sys_cmd_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(sys_cmd_frame, text="Set Verbose ON",
                  command=lambda: self.set_verbose(True)).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(sys_cmd_frame, text="Set Verbose OFF",
                  command=lambda: self.set_verbose(False)).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(sys_cmd_frame, text="Start Test Motors",
                  command=self.open_test_motors_window).grid(row=0, column=2, padx=5, pady=5)

    def process_serial_response(self, response):
        """Process responses received from the serial port."""
        if response.startswith("TEST_MOTOR_SELECTED:"):
            name = response.split(":", 1)[1].strip()
            self.selected_motor = name
            if self.test_motors_window:
                self.test_motors_window.highlight_selected_motor(name)
            self.log_test_motors_output(f"Arduino selected motor: {name}")
        elif response.startswith("VERBOSE_STATE:"):
            state = response.split(":")[1].strip()
            self.is_verbose = (state == "1")
            self.log_message(f"Verbose mode {'ON' if self.is_verbose else 'OFF'}")
        else:
            self.log_message(f"Arduino: {response}")
            if self.test_motors_window:
                self.test_motors_window.log_output(response)

    def log_message(self, message):
        """Log a message to the main output area."""
        if hasattr(self, 'output_text') and self.output_text:
            self.output_text.insert(tk.END, message + "\n")
            self.output_text.see(tk.END)

    def log_test_motors_output(self, message):
        """Log a message to the test motors window or fall back to main log."""
        if self.test_motors_window:
            self.test_motors_window.log_output(message)
        else:
            self.log_message(f"(TestMotorLog-Fallback): {message}")

    def toggle_connection(self):
        """Toggle the serial connection state."""
        if not self.is_connected:
            try:
                port = self.port_entry.get()
                success, message = self.serial_comm.connect(port)
                if success:
                    self.is_connected = True
                    self.status_label.config(text="Status: Connected", foreground="green")
                    self.connect_button.config(text="Disconnect")
                    self.log_message(f"Connected to {port}")
                else:
                    messagebox.showerror("Connection Error", message)
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            self.serial_comm.disconnect()
            self.is_connected = False
            self.status_label.config(text="Status: Disconnected", foreground="black")
            self.connect_button.config(text="Connect")
            self.log_message("Disconnected")

    def set_verbose(self, state):
        """Set verbose mode. State parameter is kept for button functionality."""
        if self.is_connected:
            self.serial_comm.send_command("TOGGLE_VERBOSE")
        else:
            self.log_message("Error: Not connected. Cannot toggle verbose mode.")

    def open_test_motors_window(self):
        """Open the test motors window if it's not already open."""
        if self.test_motors_window:
            self.test_motors_window.window.lift()
            return

        if not self.is_connected:
            messagebox.showerror("Error", "Please connect to the device first.")
            return

        self.serial_comm.send_command("START_TEST_MOTORS")
        
        # Motor names grouped by rows for the button layout
        motor_names_grouped = [
            ["EPU", "EPD", "EYR", "EYL"],
            ["WPD", "WPU", "RJL", "LJR"],
            ["LJL", "RJR", "ROLL"]
        ]

        callbacks = {
            "select_motor": self.select_test_motor,
            "home_link": self.home_link_command,
            "fine_tension": self.fine_tension_selected_motor,
            "coarse_tension": self.coarse_tension_selected_motor,
            "detension": self.detension_selected_motor,
            "step_motor": self.step_selected_motor_by_amount,
            "next_motor": self.next_test_motor,
            "exit": self.exit_test_motors
        }

        self.test_motors_window = TestMotorsWindow(
            self.root,
            motor_names_grouped,
            -1000,  # motor_test_step_min
            1000,   # motor_test_step_max
            callbacks
        )

    def select_test_motor(self, name):
        """Select a motor for testing."""
        self.selected_motor = name
        self.serial_comm.send_command(f"SELECT_MOTOR:{name}")
        self.log_test_motors_output(f"GUI selected motor: {name}")

    def home_link_command(self):
        """Set the current position as home position."""
        self.serial_comm.send_command("SET_HOME")
        self._reset_cumulative_degrees_display()
        self.log_message("Home Link activated. SET_HOME sent. Cumulative degrees reset.")
        if self.test_motors_window:
            messagebox.showinfo("Home Set", 
                              "SET_HOME command sent. Cumulative degrees in main window reset.",
                              parent=self.test_motors_window.window)

    def fine_tension_selected_motor(self):
        """Apply fine tension to the selected motor."""
        self.serial_comm.send_command("FINE_TENSION")
        self.log_test_motors_output(f"Command: FINE_TENSION for {self.selected_motor}")

    def coarse_tension_selected_motor(self):
        """Apply coarse tension to the selected motor."""
        self.serial_comm.send_command("COARSE_TENSION")
        self.log_test_motors_output(f"Command: COARSE_TENSION for {self.selected_motor}")

    def detension_selected_motor(self):
        """Detension the selected motor."""
        self.serial_comm.send_command("DETENSION")
        self.log_test_motors_output(f"Command: DETENSION for {self.selected_motor}")

    def step_selected_motor_by_amount(self):
        """Step the selected motor by the amount shown on the slider."""
        if self.test_motors_window:
            steps = self.test_motors_window.get_step_amount()
            self.serial_comm.send_command(f"STEP_MOTOR_BY:{steps}")
            self.log_test_motors_output(f"Command: STEP_MOTOR_BY:{steps} for {self.selected_motor}")

    def next_test_motor(self):
        """Select the next motor in sequence."""
        self.serial_comm.send_command("NEXT_MOTOR")
        self.log_test_motors_output("Command: NEXT_MOTOR")

    def exit_test_motors(self):
        """Exit test motors mode."""
        self.serial_comm.send_command("EXIT_TEST")
        self.log_message("Exited Test Motors mode.")
        if self.test_motors_window:
            self.test_motors_window.close()
            self.test_motors_window = None

    def _reset_cumulative_degrees_display(self):
        """Reset all cumulative degree displays to zero."""
        self.cumulative_ep_degrees_var.set(0.0)
        self.cumulative_ey_degrees_var.set(0.0)
        self.cumulative_wp_degrees_var.set(0.0)
        self.cumulative_lj_degrees_var.set(0.0)
        self.cumulative_rj_degrees_var.set(0.0)
        messagebox.showinfo("Reset", "Cumulative degree display has been reset to zero.",
                          parent=self.root)

    def update_step_size_label(self, event=None):
        """Update the step size display label when the slider changes."""
        if not hasattr(self, 'step_size_display_label'):
            return
        try:
            selected_index = int(self.current_step_size_var.get())
            if 0 <= selected_index < len(self.step_sizes):
                current_val = self.step_sizes[selected_index]
                self.step_size_display_label.config(text=f"{current_val} steps")
        except tk.TclError:
            self.step_size_display_label.config(text="N/A")

    def _create_positional_control_frame(self, parent=None):
        """Creates the Positional Control UI as an inline frame."""
        if parent is None:
            parent = self.main_frame
            
        positional_super_frame = ttk.LabelFrame(parent, text="Positional Control (Degrees)", padding="10")
        positional_super_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        parent.grid_rowconfigure(3, weight=0)

        # Input section
        input_frame = ttk.LabelFrame(positional_super_frame, text="Move by Degrees", padding="10")
        input_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ns")

        # Create input fields for each joint
        joint_inputs = [
            ("Elbow Pitch (°):", self.ep_degrees_input_var),
            ("Elbow Yaw (°):", self.ey_degrees_input_var),
            ("Wrist Pitch (°):", self.wp_degrees_input_var),
            ("Left Jaw (°):", self.lj_degrees_input_var),
            ("Right Jaw (°):", self.rj_degrees_input_var)
        ]

        for i, (label_text, var) in enumerate(joint_inputs):
            ttk.Label(input_frame, text=label_text).grid(row=i, column=0, sticky="w", pady=2)
            entry = ttk.Entry(input_frame, textvariable=var, width=10)
            entry.grid(row=i, column=1, pady=2)

        ttk.Button(input_frame, text="Move Joints by Degrees",
                  command=self._move_by_degrees_command, width=25).grid(
                      row=len(joint_inputs), column=0, columnspan=2, pady=10)

        # Cumulative display section
        cumulative_frame = ttk.LabelFrame(positional_super_frame,
                                        text="Cumulative Joint Position (Degrees from Home)",
                                        padding="10")
        cumulative_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ns")

        # Create cumulative position displays
        cumulative_vars = [
            ("Elbow Pitch:", self.cumulative_ep_degrees_var),
            ("Elbow Yaw:", self.cumulative_ey_degrees_var),
            ("Wrist Pitch:", self.cumulative_wp_degrees_var),
            ("Left Jaw:", self.cumulative_lj_degrees_var),
            ("Right Jaw:", self.cumulative_rj_degrees_var)
        ]

        for i, (label_text, var) in enumerate(cumulative_vars):
            ttk.Label(cumulative_frame, text=label_text).grid(row=i, column=0, sticky="w", pady=2)
            ttk.Label(cumulative_frame, textvariable=var, width=8,
                     anchor="e").grid(row=i, column=1, sticky="e", pady=2)
            ttk.Label(cumulative_frame, text="°").grid(row=i, column=2, sticky="w", pady=2)

        ttk.Button(cumulative_frame, text="Reset Cumulative Display",
                  command=self._reset_cumulative_degrees_display,
                  width=25).grid(row=len(cumulative_vars), column=0, columnspan=3, pady=10)

        positional_super_frame.columnconfigure(0, weight=1)
        positional_super_frame.columnconfigure(1, weight=1)

    def _create_output_area(self):
        """Create the output log area."""
        output_frame = ttk.LabelFrame(self.main_frame, text="Output Log", padding="5")
        output_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.main_frame.grid_rowconfigure(4, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)

        self.output_text = tk.Text(output_frame, height=10, width=80)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_frame.grid_rowconfigure(0, weight=1)
        output_frame.grid_columnconfigure(0, weight=1)
        
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.output_text['yscrollcommand'] = scrollbar.set

    def _move_by_degrees_command(self):
        """Handle the move joints by degrees command."""
        try:
            ep_deg = float(self.ep_degrees_input_var.get())
            ey_deg = float(self.ey_degrees_input_var.get())
            wp_deg = float(self.wp_degrees_input_var.get())
            lj_deg = float(self.lj_degrees_input_var.get())
            rj_deg = float(self.rj_degrees_input_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.",
                               parent=self.root)
            return

        # Calculate steps for all motors
        motor_steps = self.motor_controller.calculate_all_motor_steps(
            ep_deg, ey_deg, wp_deg, lj_deg, rj_deg,
            self.cumulative_wp_degrees_var.get()
        )
        
        # Create a comma-separated string of steps in enum order
        motor_steps_str = ",".join(str(steps) for steps in motor_steps)
        self.serial_comm.send_command(f"MOVE_ALL_MOTORS:{motor_steps_str}")

        # Update cumulative degrees
        self.cumulative_ep_degrees_var.set(round(self.cumulative_ep_degrees_var.get() + ep_deg, 2))
        self.cumulative_ey_degrees_var.set(round(self.cumulative_ey_degrees_var.get() + ey_deg, 2))
        self.cumulative_wp_degrees_var.set(round(self.cumulative_wp_degrees_var.get() + wp_deg, 2))
        self.cumulative_lj_degrees_var.set(round(self.cumulative_lj_degrees_var.get() + lj_deg, 2))
        self.cumulative_rj_degrees_var.set(round(self.cumulative_rj_degrees_var.get() + rj_deg, 2))
        
        # Log the movement
        self.log_message(f"Moving joints - EP:{ep_deg}°, EY:{ey_deg}°, WP:{wp_deg}°, " +
                      f"LJ:{lj_deg}°, RJ:{rj_deg}°")
        self.log_message(f"Motor steps: {motor_steps_str}")

    def get_step_size(self):
        """Get the current step size from the slider."""
        try:
            selected_index = int(self.current_step_size_var.get())
            if 0 <= selected_index < len(self.step_sizes):
                return self.step_sizes[selected_index]
        except tk.TclError:
            pass
        return 10  # Default
def main():
    root = tk.Tk()
    app = ElbowSimulatorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
