import tkinter as tk
from tkinter import ttk, messagebox, Toplevel
import math
import serial
import time
import threading
from PIL import Image, ImageTk
import os
import numpy as np
from enum import IntEnum
import json # --- NEW: Import JSON module ---

class MotorIndex(IntEnum):
    EPU = 0
    EPD = 1
    EYR = 2
    EYL = 3
    WPD = 4
    WPU = 5
    RJL = 6
    LJR = 7
    LJL = 8
    RJR = 9
    ROLL = 10

class ElbowSimulatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Elbow Control Simulator")
        self.step_size_display_label = None
        # Adjusted geometry, positional control will take up space in main window
        self.root.geometry("950x950")

        self.is_verbose = False
        self.serial_port = None
        self.is_connected = False
        
        # --- MODIFIED: Test Motors window reference ---
        self.test_motors_window = None # Will hold the Toplevel instance for test motors
        self.selected_motor = "EPD"

        self.motor_names_flat = [
            "RJR", "LJL", "LJR", "RJL", "WPU", "WPD",
            "EYR", "EYL", "EPD", "EPU"
        ]
        self.motor_names_grouped = [
            ["RJR", "LJL"], ["LJR", "RJL"], ["WPU", "WPD"],
            ["EYR", "EYL"], ["EPD", "EPU"]
        ]

        self.step_sizes = [1, 2, 5, 10, 20, 50, 100]
        self.current_step_size_var = tk.IntVar(value=self.step_sizes.index(10))

        self.motor_test_step_count_var = tk.IntVar(value=50)
        self.motor_test_step_min = -200
        self.motor_test_step_max = 200

        # Variables for Positional Control (now an inline frame)
        self.ep_degrees_input_var = tk.StringVar(value="0.0")
        self.ey_degrees_input_var = tk.StringVar(value="0.0")
        self.wp_degrees_input_var = tk.StringVar(value="0.0")
        self.lj_degrees_input_var = tk.StringVar(value="0.0")
        self.rj_degrees_input_var = tk.StringVar(value="0.0")

        self.cumulative_ep_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_ey_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_wp_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_lj_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_rj_degrees_var = tk.DoubleVar(value=0.0)

        self.canvas = tk.Canvas(self.root, borderwidth=0, background="#f0f0f0")
        self.v_scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.v_scrollbar.set)
        self.v_scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.container_frame = ttk.Frame(self.canvas)
        self.main_frame_id = self.canvas.create_window((0, 0), window=self.container_frame, anchor="nw")

        self.image_frame = ttk.Frame(self.container_frame)
        self.image_frame.grid(row=0, column=0, sticky=tk.N, padx=10, pady=10)
        self.load_logo_image()

        self.main_frame = ttk.Frame(self.container_frame, padding="10")
        self.main_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.container_frame.columnconfigure(1, weight=1)

        self.container_frame.bind("<Configure>", self.on_frame_configure)
        self.canvas.bind("<Configure>", self.on_canvas_configure)

        self.create_widgets() # This will now also create the inline positional control
        self.create_output_area()

        self.serial_thread = threading.Thread(target=self.monitor_serial, daemon=True)
        self.serial_thread.start()

        self.jaw_LJL_RJR_constants = {
            "L1": 1.457,    # Distance from center of link-1 to link-2 cable exit point
            "L2": 0.55,     # Distance from center of link-2 to link-2 cable exit point
            "r1": 0.89,     # Radius of the first guide curve (guide 1)
            "c1x": 1.6,     # x-coordinate of the center of guide 1
            "c1y": -1.5,    # y-coordinate of the center of guide 1
            "r2": 1.0,      # Radius of the second guide curve (guide 2)
            # c2x and c2y are effectively 0.0 and handled within _calculate_case2_pl
        }        

    def load_logo_image(self):
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            img_path = os.path.join(script_dir, "elbow.png")
            if not os.path.exists(img_path): img_path = "elbow.png"

            if os.path.exists(img_path):
                image = Image.open(img_path)
                max_size = 120; w, h = image.size
                if w == 0 or h == 0: raise ValueError("Image has zero width or height")
                scale = min(max_size / w, max_size / h)
                new_size = (int(w * scale), int(h * scale))
                image = image.resize(new_size, Image.Resampling.LANCZOS)
                self.logo_photo = ImageTk.PhotoImage(image)
                label = tk.Label(self.image_frame, image=self.logo_photo, background="#f0f0f0")
                label.pack()
            else:
                label = tk.Label(self.image_frame, text="Logo N/A", background="#f0f0f0", foreground="grey")
                label.pack(padx=20, pady=40)
                self.log_message(f"Warning: elbow.png not found.")
        except Exception as e:
            self.log_message(f"Error loading logo image: {e}")
            label = tk.Label(self.image_frame, text="Logo Err", background="#f0f0f0", foreground="red")
            label.pack(padx=20, pady=40)

    def on_frame_configure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def on_canvas_configure(self, event):
        self.canvas.itemconfig(self.main_frame_id, width=event.width)

    def create_widgets(self):
        # Row 0: Connection
        conn_frame = ttk.LabelFrame(self.main_frame, text="Arduino Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(conn_frame, width=10); self.port_entry.insert(0, "COM3")
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected")
        self.status_label.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        conn_frame.columnconfigure(3, weight=1)

        # Row 1: Direct Step Control
        movement_super_frame = ttk.LabelFrame(self.main_frame, text="Direct Step Control", padding="5")
        movement_super_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        # ... (D-pad, wrist, jaw controls - same as before) ...
        elbow_ctrl_frame = ttk.Frame(movement_super_frame, padding="5")
        elbow_ctrl_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N)
        ttk.Label(elbow_ctrl_frame, text="Elbow Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0,5))
        ttk.Button(elbow_ctrl_frame, text="↖", width=3, command=lambda: self.move_pitch_yaw(self.get_step_size(), -self.get_step_size())).grid(row=1, column=0, padx=1, pady=1)
        ttk.Button(elbow_ctrl_frame, text="↑", width=3, command=lambda: self.move_pitch_yaw(self.get_step_size(), 0)).grid(row=1, column=1, padx=1, pady=1)
        ttk.Button(elbow_ctrl_frame, text="↗", width=3, command=lambda: self.move_pitch_yaw(self.get_step_size(), self.get_step_size())).grid(row=1, column=2, padx=1, pady=1)
        ttk.Button(elbow_ctrl_frame, text="←", width=3, command=lambda: self.move_pitch_yaw(0, -self.get_step_size())).grid(row=2, column=0, padx=1, pady=1)
        tk.Label(elbow_ctrl_frame, text=" ").grid(row=2, column=1)
        ttk.Button(elbow_ctrl_frame, text="→", width=3, command=lambda: self.move_pitch_yaw(0, self.get_step_size())).grid(row=2, column=2, padx=1, pady=1)
        ttk.Button(elbow_ctrl_frame, text="↙", width=3, command=lambda: self.move_pitch_yaw(-self.get_step_size(), -self.get_step_size())).grid(row=3, column=0, padx=1, pady=1)
        ttk.Button(elbow_ctrl_frame, text="↓", width=3, command=lambda: self.move_pitch_yaw(-self.get_step_size(), 0)).grid(row=3, column=1, padx=1, pady=1)
        ttk.Button(elbow_ctrl_frame, text="↘", width=3, command=lambda: self.move_pitch_yaw(-self.get_step_size(), self.get_step_size())).grid(row=3, column=2, padx=1, pady=1)

        wrist_ctrl_frame = ttk.Frame(movement_super_frame, padding="5")
        wrist_ctrl_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N)
        ttk.Label(wrist_ctrl_frame, text="Wrist Pitch Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, pady=(0,5))
        ttk.Button(wrist_ctrl_frame, text="↑", width=4, command=lambda: self.move_wrist_pitch(self.get_step_size())).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(wrist_ctrl_frame, text="↓", width=4, command=lambda: self.move_wrist_pitch(-self.get_step_size())).grid(row=2, column=0, padx=2, pady=2)

        jaw_ctrl_frame = ttk.Frame(movement_super_frame, padding="5")
        jaw_ctrl_frame.grid(row=0, column=2, padx=10, pady=5, sticky=tk.N)
        ttk.Label(jaw_ctrl_frame, text="Jaw Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0,5))
        ttk.Label(jaw_ctrl_frame, text="Left Jaw:").grid(row=1, column=0, padx=2, pady=2, sticky=tk.E)
        ttk.Button(jaw_ctrl_frame, text="←", width=3, command=lambda: self.move_left_jaw(-self.get_step_size())).grid(row=1, column=1, padx=1, pady=1)
        ttk.Button(jaw_ctrl_frame, text="→", width=3, command=lambda: self.move_left_jaw(self.get_step_size())).grid(row=1, column=2, padx=1, pady=1)
        ttk.Label(jaw_ctrl_frame, text="Right Jaw:").grid(row=2, column=0, padx=2, pady=2, sticky=tk.E)
        ttk.Button(jaw_ctrl_frame, text="←", width=3, command=lambda: self.move_right_jaw(-self.get_step_size())).grid(row=2, column=1, padx=1, pady=1)
        ttk.Button(jaw_ctrl_frame, text="→", width=3, command=lambda: self.move_right_jaw(self.get_step_size())).grid(row=2, column=2, padx=1, pady=1)

        slider_frame = ttk.Frame(movement_super_frame)
        slider_frame.grid(row=1, column=0, columnspan=3, pady=(10,5))
        ttk.Label(slider_frame, text="Movement Steps:").pack(side=tk.LEFT, padx=(0,5))
        self.step_size_slider = ttk.Scale(
            slider_frame, from_=0, to=len(self.step_sizes) - 1, orient=tk.HORIZONTAL,
            variable=self.current_step_size_var, command=self.update_step_size_label, length=200)
        self.step_size_slider.pack(side=tk.LEFT, padx=(0,5)); self.step_size_slider.set(self.current_step_size_var.get())
        self.step_size_display_label = ttk.Label(slider_frame, text=f"{self.get_step_size()} steps", width=10)
        self.step_size_display_label.pack(side=tk.LEFT)
        self.update_step_size_label()
        movement_super_frame.columnconfigure(0, weight=1); movement_super_frame.columnconfigure(1, weight=1); movement_super_frame.columnconfigure(2, weight=1)

        # Row 2: System Commands (No "Open Positional Control" button here anymore)
        sys_cmd_frame = ttk.LabelFrame(self.main_frame, text="System Commands", padding="5")
        sys_cmd_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        ttk.Button(sys_cmd_frame, text="Set Verbose ON", command=lambda: self.set_verbose(True)).grid(row=0, column=0, padx=5, pady=5) # Shifted column
        ttk.Button(sys_cmd_frame, text="Set Verbose OFF", command=lambda: self.set_verbose(False)).grid(row=0, column=1, padx=5, pady=5)
        # MODIFIED: Button now calls open_test_motors_window
        ttk.Button(sys_cmd_frame, text="Start Test Motors", command=self.open_test_motors_window).grid(row=0, column=2, padx=5, pady=5)
        sys_cmd_frame.columnconfigure(3, weight=1)

        # Row 3: --- NEW: Inline Positional Control Frame ---
        self._create_positional_control_frame(self.main_frame) # Create it directly here
        
        # Row 4 will be Output Log

    def _create_positional_control_frame(self, parent):
        """Creates the Positional Control UI as an inline frame."""
        positional_super_frame = ttk.LabelFrame(parent, text="Positional Control (Degrees)", padding="10")
        positional_super_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        parent.grid_rowconfigure(3, weight=0) # Adjust weight as needed, 0 means it won't expand much

        # Input section
        input_frame = ttk.LabelFrame(positional_super_frame, text="Move by Degrees", padding="10")
        input_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ns")

        ttk.Label(input_frame, text="Elbow Pitch (°):").grid(row=0, column=0, sticky="w", pady=2)
        self.ep_degrees_entry = ttk.Entry(input_frame, textvariable=self.ep_degrees_input_var, width=10)
        self.ep_degrees_entry.grid(row=0, column=1, pady=2)

        ttk.Label(input_frame, text="Elbow Yaw (°):").grid(row=1, column=0, sticky="w", pady=2)
        self.ey_degrees_entry = ttk.Entry(input_frame, textvariable=self.ey_degrees_input_var, width=10)
        self.ey_degrees_entry.grid(row=1, column=1, pady=2)

        ttk.Label(input_frame, text="Wrist Pitch (°):").grid(row=2, column=0, sticky="w", pady=2)
        self.wp_degrees_entry = ttk.Entry(input_frame, textvariable=self.wp_degrees_input_var, width=10)
        self.wp_degrees_entry.grid(row=2, column=1, pady=2)

        ttk.Label(input_frame, text="Left Jaw (°):").grid(row=3, column=0, sticky="w", pady=2)
        self.lj_degrees_entry = ttk.Entry(input_frame, textvariable=self.lj_degrees_input_var, width=10)
        self.lj_degrees_entry.grid(row=3, column=1, pady=2)

        ttk.Label(input_frame, text="Right Jaw (°):").grid(row=4, column=0, sticky="w", pady=2)
        self.rj_degrees_entry = ttk.Entry(input_frame, textvariable=self.rj_degrees_input_var, width=10)
        self.rj_degrees_entry.grid(row=4, column=1, pady=2)

        ttk.Button(input_frame, text="Move Joints by Degrees", command=self._move_by_degrees_command, width=25).grid(row=5, column=0, columnspan=2, pady=10)

        # Cumulative display section
        cumulative_frame = ttk.LabelFrame(positional_super_frame, text="Cumulative Joint Position (Degrees from Home)", padding="10")
        cumulative_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ns")

        ttk.Label(cumulative_frame, text="Elbow Pitch:").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ep_degrees_var, width=8, anchor="e").grid(row=0, column=1, sticky="e", pady=2)
        ttk.Label(cumulative_frame, text="°").grid(row=0, column=2, sticky="w", pady=2)

        ttk.Label(cumulative_frame, text="Elbow Yaw:").grid(row=1, column=0, sticky="w", pady=2)
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ey_degrees_var, width=8, anchor="e").grid(row=1, column=1, sticky="e", pady=2)
        ttk.Label(cumulative_frame, text="°").grid(row=1, column=2, sticky="w", pady=2)

        ttk.Label(cumulative_frame, text="Wrist Pitch:").grid(row=2, column=0, sticky="w", pady=2)
        ttk.Label(cumulative_frame, textvariable=self.cumulative_wp_degrees_var, width=8, anchor="e").grid(row=2, column=1, sticky="e", pady=2)
        ttk.Label(cumulative_frame, text="°").grid(row=2, column=2, sticky="w", pady=2)

        ttk.Label(cumulative_frame, text="Left Jaw:").grid(row=3, column=0, sticky="w", pady=2)
        ttk.Label(cumulative_frame, textvariable=self.cumulative_lj_degrees_var, width=8, anchor="e").grid(row=3, column=1, sticky="e", pady=2)
        ttk.Label(cumulative_frame, text="°").grid(row=3, column=2, sticky="w", pady=2)

        ttk.Label(cumulative_frame, text="Right Jaw:").grid(row=4, column=0, sticky="w", pady=2)
        ttk.Label(cumulative_frame, textvariable=self.cumulative_rj_degrees_var, width=8, anchor="e").grid(row=4, column=1, sticky="e", pady=2)
        ttk.Label(cumulative_frame, text="°").grid(row=4, column=2, sticky="w", pady=2)
        
        ttk.Button(cumulative_frame, text="Reset Cumulative Display", command=self._reset_cumulative_degrees_display, width=25).grid(row=5, column=0, columnspan=3, pady=10)
        
        positional_super_frame.columnconfigure(0, weight=1) # Distribute space between input and cumulative
        positional_super_frame.columnconfigure(1, weight=1)


    def update_step_size_label(self, event=None):
        if not hasattr(self, 'step_size_display_label') or not self.step_size_display_label: return
        selected_index = int(self.current_step_size_var.get())
        current_val = self.step_sizes[selected_index] if 0 <= selected_index < len(self.step_sizes) else "N/A"
        self.step_size_display_label.config(text=f"{current_val} steps" if current_val != "N/A" else "N/A")

    def create_output_area(self):
        output_frame = ttk.LabelFrame(self.main_frame, text="Output Log", padding="5")
        # MODIFIED: Row index for output area, now row 4
        output_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.main_frame.grid_rowconfigure(4, weight=1) # Output log should expand
        self.main_frame.grid_columnconfigure(0, weight=1)

        self.output_text = tk.Text(output_frame, height=10, width=80)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_frame.grid_rowconfigure(0, weight=1); output_frame.grid_columnconfigure(0, weight=1)
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.output_text['yscrollcommand'] = scrollbar.set

    def toggle_connection(self):
        # ... (same as before) ...
        if not self.is_connected:
            try:
                port = self.port_entry.get()
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                time.sleep(2) # Allow Arduino to reset
                self.is_connected = True
                self.status_label.config(text=f"Status: Connected ({port})", foreground="green")
                self.connect_button.config(text="Disconnect")
                self.log_message(f"Connected to {port}")
            except serial.SerialException as e:
                messagebox.showerror("Connection Error", f"Failed to connect to {port}: {str(e)}", parent=self.root)
                self.status_label.config(text="Status: Error", foreground="red")
        else:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.status_label.config(text="Status: Disconnected", foreground="black")
            self.connect_button.config(text="Connect")
            self.log_message("Disconnected")


    def send_command(self, command):
        # ... (same as before) ...
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.log_message(f"Sent: {command}")
            except serial.SerialException as e:
                self.log_message(f"Error sending command: {e}. Lost connection.")
                # Schedule toggle_connection to run in the main thread
                self.root.after(0, self.toggle_connection)
            except Exception as e: # Other exceptions like port not open
                self.log_message(f"Unexpected error sending command: {e}")
                if self.is_connected: # If we thought we were connected, attempt to fix state
                    self.root.after(0, self.toggle_connection)
        elif not self.is_connected:
            self.log_message("Error: Not connected. Cannot send command.")


    def monitor_serial(self):
        while True:
            if self.is_connected and self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        response = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                        if response:
                            self.root.after(0, self.process_serial_response, response)
                except serial.SerialException: # Port has been closed or disconnected
                    if self.is_connected: # only toggle if we thought we were connected
                        self.root.after(0, self.log_message, "Error: Lost connection during serial monitoring.")
                        self.root.after(0, self.toggle_connection) # This will set is_connected to False
                except Exception as e: # Other unexpected errors
                    self.root.after(0, self.log_message, f"Unexpected serial monitoring error: {e}")
            time.sleep(0.1) # Reduce CPU usage

    def process_serial_response(self, response):
        if response.startswith("TEST_MOTOR_SELECTED:"):
            name = response.split(":", 1)[1]
            self.selected_motor = name
            if self.test_motors_window and self.test_motors_window.winfo_exists():
                self.highlight_selected_motor_in_window()
            self.log_test_motors_output(f"Arduino selected motor: {name}")
        elif response.startswith("VERBOSE_STATE:"):
            state = response.split(":")[1].strip()
            self.is_verbose = (state == "1")
            self.log_message(f"Verbose mode {'ON' if self.is_verbose else 'OFF'}")
        else:
            self.log_message(f"Arduino: {response}")
            if self.test_motors_window and self.test_motors_window.winfo_exists():
                if hasattr(self, 'test_motors_output_text_widget') and self.test_motors_output_text_widget:
                    self.log_test_motors_output(f"Arduino: {response}")


    def log_message(self, message):
        # ... (same as before) ...
        if hasattr(self, 'output_text') and self.output_text:
            self.output_text.insert(tk.END, message + "\n")
            self.output_text.see(tk.END)


    def move_pitch_yaw(self, pitch_step_delta, yaw_step_delta): # sign indicates direction
        # ... (same as before, sends steps) ...
        actual_pitch_steps = int(pitch_step_delta)
        actual_yaw_steps = int(yaw_step_delta)

        if actual_pitch_steps != 0 and actual_yaw_steps != 0:
            self.send_command(f"MOVE_EP_EY_REL:{actual_pitch_steps},{actual_yaw_steps}")
        elif actual_pitch_steps != 0:
            self.send_command(f"MOVE_EP_REL:{actual_pitch_steps}")
        elif actual_yaw_steps != 0:
            self.send_command(f"MOVE_EY_REL:{actual_yaw_steps}")

    def move_wrist_pitch(self, step_delta):
        steps = int(step_delta)
        if steps != 0: self.send_command(f"MOVE_WP_REL:{steps}")

    def move_left_jaw(self, step_delta):
        steps = int(step_delta)
        if steps != 0: self.send_command(f"MOVE_LJ_REL:{steps}")

    def move_right_jaw(self, step_delta):
        steps = int(step_delta)
        if steps != 0: self.send_command(f"MOVE_RJ_REL:{steps}")

    def set_verbose(self, state):
        """Set verbose mode. State parameter is kept for button functionality."""
        self.send_command("TOGGLE_VERBOSE")

    def open_test_motors_window(self):
        if self.test_motors_window and self.test_motors_window.winfo_exists():
            self.test_motors_window.lift()
            return

        self.send_command("START_TEST_MOTORS") # Notify Arduino

        self.test_motors_window = Toplevel(self.root)
        self.test_motors_window.title("Test Motors Control")
        self.test_motors_window.geometry("750x450") # Adjusted size for content
        self.test_motors_window.resizable(True, True)
        self.test_motors_window.transient(self.root) # Keep it above main window

        # Create the content for the Toplevel window
        content_frame = ttk.Frame(self.test_motors_window, padding="10")
        content_frame.pack(expand=True, fill="both")

        # Left: Motor buttons
        motor_buttons_frame = ttk.Frame(content_frame)
        motor_buttons_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N)
        self.motor_buttons_in_window = {} # Store buttons for this window
        style = ttk.Style() # Ensure style is available
        style.configure("SelectedMotor.TButton", background="#0078D7", foreground="white", font=("Segoe UI", 9, "bold"))
        style.map("SelectedMotor.TButton", background=[('active', '#005a9e')], foreground=[('active', 'white')])
        style.configure("TButton", font=("Segoe UI", 9))

        for r, row_names in enumerate(self.motor_names_grouped):
            for c, name in enumerate(row_names):
                btn = ttk.Button(motor_buttons_frame, text=name, width=6,
                                 command=lambda n=name: self.select_test_motor_in_window(n))
                btn.grid(row=r, column=c, pady=3, padx=3)
                self.motor_buttons_in_window[name] = btn

        # Middle: Controls for selected motor
        motor_controls_frame = ttk.Frame(content_frame)
        motor_controls_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N)

        ttk.Button(motor_controls_frame, text="Home Link (Set Home)", width=20, command=self._home_link_command).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Fine Tension", width=20, command=self.fine_tension_selected_motor).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Coarse Tension", width=20, command=self.coarse_tension_selected_motor).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Detension", width=20, command=self.detension_selected_motor).pack(pady=3, fill=tk.X)

        step_motor_manual_frame = ttk.Frame(motor_controls_frame)
        step_motor_manual_frame.pack(pady=3, fill=tk.X)
        ttk.Button(step_motor_manual_frame, text="Step Motor", width=15, command=self.step_selected_motor_by_amount).pack(side=tk.LEFT,pady=3)
        
        # Ensure motor_test_step_slider and display_label are attributes of self if accessed elsewhere
        self.motor_test_step_slider_widget = ttk.Scale(
            step_motor_manual_frame, from_=self.motor_test_step_min, to=self.motor_test_step_max, orient=tk.HORIZONTAL,
            variable=self.motor_test_step_count_var, command=self.update_motor_test_step_label_in_window, length=100)
        self.motor_test_step_slider_widget.pack(side=tk.LEFT, padx=5, pady=(3,0), fill=tk.X, expand=True)
        self.motor_test_step_display_label_widget = ttk.Label(step_motor_manual_frame, text=f"{self.motor_test_step_count_var.get()} steps", width=10)
        self.motor_test_step_display_label_widget.pack(side=tk.LEFT, padx=5, pady=(3,0))
        self.update_motor_test_step_label_in_window() # Initialize

        ttk.Button(motor_controls_frame, text="Next Motor", width=20, command=self.next_test_motor_in_window).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Exit Test Mode", width=20, command=self.exit_test_motors_window).pack(pady=3, fill=tk.X)

        # Right: Output area for test motors
        test_output_frame = ttk.Frame(content_frame)
        test_output_frame.grid(row=0, column=2, padx=10, pady=5, sticky=(tk.N, tk.S, tk.E, tk.W))
        content_frame.columnconfigure(2, weight=1)
        content_frame.rowconfigure(0, weight=1)

        ttk.Label(test_output_frame, text="Test Motor Log:").grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0,2))
        # Store the text widget on self to be accessible by log_test_motors_output
        self.test_motors_output_text_widget = tk.Text(test_output_frame, height=15, width=45)
        self.test_motors_output_text_widget.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        test_scrollbar = ttk.Scrollbar(test_output_frame, orient=tk.VERTICAL, command=self.test_motors_output_text_widget.yview)
        test_scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        test_output_frame.rowconfigure(1, weight=1)
        test_output_frame.columnconfigure(0, weight=1)
        self.test_motors_output_text_widget['yscrollcommand'] = test_scrollbar.set

        self.highlight_selected_motor_in_window() # Initial highlight
        self.select_test_motor_in_window(self.selected_motor) # Ensure Arduino is in sync

        self.test_motors_window.protocol("WM_DELETE_WINDOW", self.exit_test_motors_window)
        self.test_motors_window.grab_set() # Make it modal (optional, remove if not desired)


    def update_motor_test_step_label_in_window(self, event=None):
        if hasattr(self, 'motor_test_step_display_label_widget') and self.motor_test_step_display_label_widget.winfo_exists():
            steps = int(self.motor_test_step_count_var.get())
            self.motor_test_step_display_label_widget.config(text=f"{steps} steps")

    def get_motor_test_step_amount(self):
        try: return int(self.motor_test_step_count_var.get())
        except tk.TclError: return 0

    def step_selected_motor_by_amount(self): # Command logic is the same
        steps = self.get_motor_test_step_amount()
        self.send_command(f"STEP_MOTOR_BY:{steps}")
        self.log_test_motors_output(f"Command: STEP_MOTOR_BY:{steps} for {self.selected_motor}")

    def highlight_selected_motor_in_window(self):
        if not (hasattr(self, 'motor_buttons_in_window') and self.motor_buttons_in_window): return
        for name, btn in self.motor_buttons_in_window.items():
            if btn.winfo_exists():
                btn.config(style="SelectedMotor.TButton" if name == self.selected_motor else "TButton")

    def select_test_motor_in_window(self, name): # Operates on Toplevel window
        self.selected_motor = name
        self.highlight_selected_motor_in_window()
        self.send_command(f"SELECT_MOTOR:{name}")
        self.log_test_motors_output(f"GUI selected motor: {name}")

    def fine_tension_selected_motor(self): # Command logic is the same
        self.send_command("FINE_TENSION")
        self.log_test_motors_output(f"Command: FINE_TENSION for {self.selected_motor}")

    def coarse_tension_selected_motor(self): # Command logic is the same
        self.send_command("COARSE_TENSION")
        self.log_test_motors_output(f"Command: COARSE_TENSION for {self.selected_motor}")

    def detension_selected_motor(self): # Command logic is the same
        self.send_command("DETENSION")
        self.log_test_motors_output(f"Command: DETENSION for {self.selected_motor}")

    def next_test_motor_in_window(self): # Command logic is the same, Arduino responds
        self.send_command("NEXT_MOTOR")
        self.log_test_motors_output("Command: NEXT_MOTOR")

    def exit_test_motors_window(self): # Closes the Toplevel window
        self.send_command("EXIT_TEST")
        self.log_message("Exited Test Motors mode.") # Log to main window
        if self.test_motors_window and self.test_motors_window.winfo_exists():
            self.test_motors_window.grab_release() # Release grab if it was set
            self.test_motors_window.destroy()
        self.test_motors_window = None
        # Clean up widgets specific to the test motor window if necessary
        if hasattr(self, 'test_motors_output_text_widget'):
            del self.test_motors_output_text_widget
        if hasattr(self, 'motor_buttons_in_window'):
            del self.motor_buttons_in_window


    def log_test_motors_output(self, message):
        # Logs to the Text widget within the Test Motors Toplevel window
        if hasattr(self, 'test_motors_output_text_widget') and \
           self.test_motors_output_text_widget and \
           self.test_motors_output_text_widget.winfo_exists():
            self.test_motors_output_text_widget.insert(tk.END, message + "\n")
            self.test_motors_output_text_widget.see(tk.END)
        else: # Fallback to main log if Toplevel/widget isn't ready/available
            self.log_message(f"(TestMotorLog-Fallback): {message}")

    def _calculate_px_py(self, q2_rad, L1_const, L2_const):
        """
        Calculates the cable exit point (px, py) from link-2. (Equation 2)
        q2_rad: Angle q2 in radians.
        L1_const, L2_const: Length constants in mm
        """
        # Handle potential domain error for asin if L2_const > L1_const
        val_for_asin_arg = L2_const / L1_const
        if not (-1.0 <= val_for_asin_arg <= 1.0):
            # This case should ideally not happen with valid geometry.
            # Log an error or return a specific failure indicator.
            # For now, let's clamp it, though this indicates a geometry issue.
            # self.log_message(f"Warning: asin argument {val_for_asin_arg} out of range. Clamping.")
            val_for_asin_arg = max(-1.0, min(1.0, val_for_asin_arg))
        
        angle_offset = math.asin(val_for_asin_arg)
        px_val = L1_const * math.cos(q2_rad - angle_offset)
        py_val = L1_const * math.sin(q2_rad - angle_offset)
        return px_val, py_val

    def _calculate_case1_pl(self, px_val, py_val, r1_c, c1x_c, c1y_c):
        """
        Calculates Path Length for Case 1: PL(q2) = l1 + s1.
        """
        l1_val_squared_arg = (px_val - c1x_c)**2 + (py_val - c1y_c)**2 - r1_c**2
        if l1_val_squared_arg < 0:
            # This can happen if P is inside C1 or r1 is too large for the geometry
            # self.log_message(f"Warning: Negative sqrt argument for l1 in Case 1: {l1_val_squared_arg}")
            l1_val_squared_arg = 0 # Avoid math domain error, but indicates issue
        l1_val = math.sqrt(l1_val_squared_arg)

        # atan(l1/r1)
        if r1_c == 0: # Avoid division by zero
            alpha1_val = math.pi / 2 if l1_val > 0 else 0.0
        else:
            alpha1_val = math.atan(l1_val / r1_c)

        # atan((px-c1x)/(py-c1y))
        numerator_beta1 = px_val - c1x_c
        denominator_beta1 = py_val - c1y_c
        if denominator_beta1 == 0: # Avoid division by zero for beta1
             beta1_val = math.copysign(math.pi / 2, numerator_beta1) if numerator_beta1 != 0 else 0.0
        else:
            beta1_val = math.atan(numerator_beta1 / denominator_beta1)
        
        gamma1_val = (math.pi / 2) - alpha1_val - abs(beta1_val)
        s1_val = r1_c * abs(gamma1_val)
        return l1_val + s1_val

    def _calculate_case2_pl(self, px_val, py_val, r1_c, c1x_c, c1y_c, r2_c): # Removed c2x_c, c2y_c
        """
        Calculates Path Length for Case 2: PL(q2) = l2 + 2*s2 + l3 + s3.
        c2x and c2y are assumed to be 0.0.
        """
        c2x_internal = 0.0
        c2y_internal = 0.0

        l3_val_squared_arg = (px_val - c2x_internal)**2 + (py_val - c2y_internal)**2 - r2_c**2
        if l3_val_squared_arg < 0:
            # self.log_message(f"Warning: Negative sqrt argument for l3 in Case 2: {l3_val_squared_arg}")
            l3_val_squared_arg = 0
        l3_val = math.sqrt(l3_val_squared_arg)

        if r2_c == 0:
            alpha3_val = math.pi / 2 if l3_val > 0 else 0.0
        else:
            alpha3_val = math.atan(l3_val / r2_c)
        
        beta3_val = math.atan2(py_val - c2y_internal, px_val - c2x_internal) # Use (py - 0), (px - 0)

        # User's gamma3 logic from provided code
        # if beta3_val < 0: 
        #     gamma3_val = (math.pi / 2) - alpha3_val + beta3_val
        # else: # beta3_val >= 0
        #     gamma3_val = (-math.pi / 2) - alpha3_val + beta3_val
        gamma3_val = beta3_val - alpha3_val # Using the simplified one from user's code for now

        s3_val = r2_c * abs(gamma3_val)

        dist_sq_c1_c2 = (c1x_c - c2x_internal)**2 + (c1y_c - c2y_internal)**2
        l2_squared_arg = dist_sq_c1_c2 - (r1_c + r2_c)**2
        if l2_squared_arg < 0:
            # self.log_message(f"Warning: Negative sqrt argument for l2 in Case 2: {l2_squared_arg}")
            l2_squared_arg = 0
        l2_val = math.sqrt(l2_squared_arg)

        denom_alpha2_arg = dist_sq_c1_c2
        if denom_alpha2_arg <= 0 : # or (r1_c + r2_c)**2 > dist_sq_c1_c2
             # This implies C1 and C2 are overlapping or r1+r2 is too large
             # self.log_message(f"Warning: Invalid geometry for alpha2 calculation in Case 2.")
             alpha2_val = 0 # Default, or handle error
        else:
            denom_alpha2 = math.sqrt(denom_alpha2_arg)
            arg_acos_alpha2 = (r1_c + r2_c) / denom_alpha2
            if not (-1.0 <= arg_acos_alpha2 <= 1.0):
                # self.log_message(f"Warning: acos argument {arg_acos_alpha2} out of range for alpha2. Clamping.")
                arg_acos_alpha2 = max(-1.0, min(1.0, arg_acos_alpha2))
            alpha2_val = math.acos(arg_acos_alpha2)

        den_beta2_val = abs(c1y_c - c2y_internal)
        if den_beta2_val == 0:
            beta2_val = math.pi / 2 if abs(c1x_c - c2x_internal) > 0 else 0.0
        else:
            beta2_val = math.atan(abs(c1x_c - c2x_internal) / den_beta2_val)
        
        gamma2_val = (math.pi / 2) - alpha2_val - beta2_val
        s2_component_from_eq17 = r2_c * abs(gamma2_val)
        
        return l2_val + (2 * s2_component_from_eq17) + l3_val + s3_val

    def _calculate_pl_value(self, q2_deg_val, constants):
        """
        Calculates the path length PL for LJL/RJR cables for a given q2.
        `constants` is a dictionary like self.jaw_LJL_RJR_constants
        """
        L1_c = constants["L1"]
        L2_c = constants["L2"]
        r1_c = constants["r1"]
        c1x_c = constants["c1x"]
        c1y_c = constants["c1y"]
        r2_c = constants["r2"]

        q2_rad_val = math.radians(q2_deg_val)
        px_val, py_val = self._calculate_px_py(q2_rad_val, L1_c, L2_c)
        
        if (c1x_c - px_val) < r1_c: # Condition from user's code
            pl_res = self._calculate_case1_pl(px_val, py_val, r1_c, c1x_c, c1y_c)
        else:
            pl_res = self._calculate_case2_pl(px_val, py_val, r1_c, c1x_c, c1y_c, r2_c) # Pass only r2_c
            
        return pl_res


    def get_step_size(self):
        try:
            selected_index = int(self.current_step_size_var.get())
            if 0 <= selected_index < len(self.step_sizes): return self.step_sizes[selected_index]
        except tk.TclError: pass
        return 10 # Default

    # --- Positional Control Logic (now for inline frame) ---
    def _move_by_degrees_command(self):
        try:
            ep_deg = float(self.ep_degrees_input_var.get())
            ey_deg = float(self.ey_degrees_input_var.get())
            wp_deg = float(self.wp_degrees_input_var.get())
            lj_deg = float(self.lj_degrees_input_var.get())
            rj_deg = float(self.rj_degrees_input_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.", parent=self.root)
            return

        # Calculate steps for all motors
        motor_steps = self._calculate_all_motor_steps_from_degrees(ep_deg, ey_deg, wp_deg, lj_deg, rj_deg)
        
        # Create a comma-separated string of steps in enum order
        motor_steps_str = ",".join(str(steps) for steps in motor_steps)
        self.send_command(f"MOVE_ALL_MOTORS:{motor_steps_str}")

        # Update cumulative degrees
        self.cumulative_ep_degrees_var.set(round(self.cumulative_ep_degrees_var.get() + ep_deg, 2))
        self.cumulative_ey_degrees_var.set(round(self.cumulative_ey_degrees_var.get() + ey_deg, 2))
        self.cumulative_wp_degrees_var.set(round(self.cumulative_wp_degrees_var.get() + wp_deg, 2))
        self.cumulative_lj_degrees_var.set(round(self.cumulative_lj_degrees_var.get() + lj_deg, 2))
        self.cumulative_rj_degrees_var.set(round(self.cumulative_rj_degrees_var.get() + rj_deg, 2))

    def _home_link_command(self): # Called from Test Motors window, but affects main UI's cumulative display
        self.send_command("SET_HOME")
        self._reset_cumulative_degrees_display() # This now resets labels in the inline frame
        self.log_message("Home Link activated. SET_HOME sent. Cumulative degrees reset.")
        # If the test motors window is open, you might want to give it focus or show a message there.
        if self.test_motors_window and self.test_motors_window.winfo_exists():
            messagebox.showinfo("Home Set", "SET_HOME command sent. Cumulative degrees in main window reset.", parent=self.test_motors_window)


    def _reset_cumulative_degrees_display(self): # Resets vars tied to inline frame labels
        self.cumulative_ep_degrees_var.set(0.0)
        self.cumulative_ey_degrees_var.set(0.0)
        self.cumulative_wp_degrees_var.set(0.0)
        self.cumulative_lj_degrees_var.set(0.0)
        self.cumulative_rj_degrees_var.set(0.0)
        messagebox.showinfo("Reset", "Cumulative degree display has been reset to zero.", parent=self.root)


    # --- Placeholder Degree-to-Step Conversion Functions ---
    # YOU NEED TO IMPLEMENT THE ACTUAL LOGIC HERE
    # ... (get_steps_elbow_pitch, get_steps_elbow_yaw, etc. - same as before) ...

    def get_steps_elbow_pitch(self, degrees_ep):
        # REMEMBER: RETURN FORMAT IS [EPU, EPD, EYR, EYL, WPD, WPU, RJL, LJR, LJL, RJR, ROLL]
        steps_ep_calc = int(degrees_ep * 100) # Assuming 100 steps per degree for EP
        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.EPU] = steps_ep_calc
        motor_steps[MotorIndex.EPD] = -steps_ep_calc
        return motor_steps

    def get_steps_elbow_yaw(self, degrees_ey):
        # REMEMBER: RETURN FORMAT IS [EPU, EPD, EYR, EYL, WPD, WPU, RJL, LJR, LJL, RJR, ROLL]
        steps_ey_calc = int(degrees_ey * 100) # Assuming 100 steps per degree for EY
        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.EYR] = steps_ey_calc
        motor_steps[MotorIndex.EYL] = -steps_ey_calc
        return motor_steps
    
    def get_steps_wrist_pitch(self, degrees_wp):
        ##primary cables:
        rad_wp = math.radians(degrees_wp)
        mm_wp = rad_wp*1.7
        steps_wp = int(mm_wp*(200/0.3))

        motor_steps = [0] * len(MotorIndex)
        
        ##auxiliary cables: 

        current_q_wrist_deg = 90+self.cumulative_wp_degrees_var.get()
        target_q_wrist_deg = current_q_wrist_deg + degrees_wp

        # Calculate for Left Jaw side (shortest when wrist is all the way up, theta = 0)
        try:
            L_current_LJL_LJR = self._calculate_pl_value(current_q_wrist_deg, self.jaw_LJL_RJR_constants)
            L_target_LJL_LJR = self._calculate_pl_value(target_q_wrist_deg, self.jaw_LJL_RJR_constants)

            L_current_RJ = self._calculate_pl_value(180.0-current_q_wrist_deg, self.jaw_LJL_RJR_constants)
            L_target_RJ = self._calculate_pl_value(180-target_q_wrist_deg, self.jaw_LJL_RJR_constants)           
        except Exception as e:
            self.log_message(f"Error in PL calculation: {e}")
            L_current_LJL_LJR = 0
            L_target_LJL_LJR = 0    
            L_current_RJ = 0
            L_target_RJ = 0  

        delta_LJ = L_target_LJL_LJR - L_current_LJL_LJR
        delta_RJ = L_target_RJ - L_current_RJ

        self.log_message(f"Wrist pitch delta: {degrees_wp}, current_abs_wp: {current_q_wrist_deg}, target_abs_wp: {target_q_wrist_deg}")
        self.log_message(f"L_current_LJL_LJR: {L_current_LJL_LJR:.4f}, L_target_LJL_LJR: {L_target_LJL_LJR:.4f}, delta_L: {delta_LJ:.4f}")
        self.log_message(f"L_current_RJ: {L_current_RJ:.4f}, L_target_RJ: {L_target_RJ:.4f}, delta_L: {delta_RJ:.4f}")

        #positive steps = cable shortening! so flip the signs, because a negative delta length = cable shortening
        steps_lj = -int(delta_LJ*(200/0.3))
        steps_rj = -int(delta_RJ*(200/0.3))

        motor_steps[MotorIndex.WPD] = -steps_wp
        motor_steps[MotorIndex.WPU] = steps_wp
        motor_steps[MotorIndex.RJL] = steps_rj
        motor_steps[MotorIndex.LJR] = steps_lj
        motor_steps[MotorIndex.LJL] = steps_lj
        motor_steps[MotorIndex.RJR] = steps_rj
        return motor_steps

    def get_steps_left_jaw(self, degrees_lj):
        #radius = 1.35
        rad_lj = math.radians(degrees_lj)
        mm_lj = rad_lj*1.35
        steps_lj = int(mm_lj * (200/0.3)) 
        
        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.LJR] = steps_lj
        motor_steps[MotorIndex.LJL] = -steps_lj
        return motor_steps

    def get_steps_right_jaw(self, degrees_rj):
        #radius = 1.35
        rad_rj = math.radians(degrees_rj)
        mm_rj = rad_rj*1.35
        steps_rj = int(mm_rj * (200/0.3)) 

        motor_steps = [0] * len(MotorIndex)
        motor_steps[MotorIndex.RJL] = -steps_rj
        motor_steps[MotorIndex.RJR] = steps_rj
        return motor_steps

    def _calculate_all_motor_steps_from_degrees(self, ep_deg, ey_deg, wp_deg, lj_deg, rj_deg):
        # Initialize a list to store the total steps for each motor
        total_motor_steps = [0] * len(MotorIndex)

        joint_processors = {
            "EP": (ep_deg, self.get_steps_elbow_pitch),
            "EY": (ey_deg, self.get_steps_elbow_yaw),
            "WP": (wp_deg, self.get_steps_wrist_pitch),
            "LJ": (lj_deg, self.get_steps_left_jaw),
            "RJ": (rj_deg, self.get_steps_right_jaw),
        }

        for joint_name, (degree_delta, get_steps_function) in joint_processors.items():
            if degree_delta != 0:
                try:
                    self.log_message(f"Calculating steps for {joint_name} with delta: {degree_delta}")
                    joint_specific_motor_steps = get_steps_function(degree_delta)
                    for idx in MotorIndex:
                        total_motor_steps[idx] += joint_specific_motor_steps[idx]
                    self.log_message(f"Steps from {joint_name}: {joint_specific_motor_steps}")
                except Exception as e:
                    self.log_message(f"Error calling {get_steps_function.__name__} for {joint_name}: {e}")
        
        final_integer_steps = [int(round(s)) for s in total_motor_steps]
        self.log_message(f"Final calculated steps for all motors: {final_integer_steps}")
        return final_integer_steps


def main():
    root = tk.Tk()
    style = ttk.Style(root)
    themes = style.theme_names()
    if "clam" in themes: style.theme_use("clam")
    elif "vista" in themes: style.theme_use("vista")
    elif "aqua" in themes: style.theme_use("aqua")
    
    app = ElbowSimulatorGUI(root)
    
    def on_closing():
        if app.is_connected:
            app.log_message("Closing connection before exit...")
            if app.test_motors_window and app.test_motors_window.winfo_exists(): # If test window is open
                 app.send_command("EXIT_TEST") # Try to gracefully exit test mode
                 time.sleep(0.1) # Give command time to send
            if app.serial_port and app.serial_port.is_open:
                app.serial_port.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()