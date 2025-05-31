import tkinter as tk
from tkinter import ttk, messagebox, Toplevel
import math # Not directly used in this snippet, but often useful
import serial
import time
import threading
from PIL import Image, ImageTk
import os

class ElbowSimulatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Elbow Control Simulator")
        self.step_size_display_label = None
        self.root.geometry("950x850") # Increased height for new controls potentially

        # Initialize variables
        self.current_ep_angle = 0.0 # This might become redundant with new tracking
        self.current_ey_angle = 0.0 # This might become redundant with new tracking
        self.is_verbose = False
        self.serial_port = None
        self.is_connected = False
        self.test_motors_frame = None
        self.selected_motor = "EPD" # Default selected motor

        # Motor names in the order expected by the 10-element array for MOVE_ALL_MOTORS
        # [RJR, LJL, LJR, RJL, WPU, WPD, EYR, EYL, EPD, EPU]
        self.motor_names_flat = [
            "RJR", "LJL", # Right Jaw pair
            "LJR", "RJL", # Left Jaw pair
            "WPU", "WPD", # Wrist Pitch pair
            "EYR", "EYL", # Elbow Yaw pair
            "EPD", "EPU"  # Elbow Pitch pair
        ]
        # Used for button layout in Test Motors
        self.motor_names_grouped = [
            ["RJR", "LJL"],
            ["LJR", "RJL"],
            ["WPU", "WPD"],
            ["EYR", "EYL"],
            ["EPD", "EPU"]
        ]

        self.step_sizes = [1, 2, 5, 10, 20, 50, 100] # Define the discrete step sizes for movement
        self.current_step_size_var = tk.IntVar(value=self.step_sizes.index(10)) # Default to 10 steps

        self.motor_test_step_count_var = tk.IntVar(value=50)
        self.motor_test_step_min = -200
        self.motor_test_step_max = 200

        # --- NEW: Variables for Degrees Input Window ---
        self.degrees_input_window = None
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
        # --- END NEW ---

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

        self.create_widgets()
        self.create_output_area()

        self.serial_thread = threading.Thread(target=self.monitor_serial, daemon=True)
        self.serial_thread.start()

    def load_logo_image(self):
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            img_path = os.path.join(script_dir, "elbow.png")
            if not os.path.exists(img_path):
                img_path = "elbow.png"

            if os.path.exists(img_path):
                image = Image.open(img_path)
                max_size = 120
                w, h = image.size
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
                self.log_message(f"Warning: elbow.png not found in {script_dir} or current directory.")
        except Exception as e:
            self.log_message(f"Error loading logo image: {e}")
            label = tk.Label(self.image_frame, text="Logo Err", background="#f0f0f0", foreground="red")
            label.pack(padx=20, pady=40)

    def on_frame_configure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def on_canvas_configure(self, event):
        canvas_width = event.width
        self.canvas.itemconfig(self.main_frame_id, width=canvas_width)

    def create_widgets(self):
        conn_frame = ttk.LabelFrame(self.main_frame, text="Arduino Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(conn_frame, width=10)
        self.port_entry.insert(0, "COM3")
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected")
        self.status_label.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        conn_frame.columnconfigure(3, weight=1)

        movement_super_frame = ttk.LabelFrame(self.main_frame, text="Direct Step Control", padding="5")
        movement_super_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
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
        # MODIFIED: Label for step size
        ttk.Label(slider_frame, text="Movement Steps:").pack(side=tk.LEFT, padx=(0,5))
        self.step_size_slider = ttk.Scale(
            slider_frame, from_=0, to=len(self.step_sizes) - 1, orient=tk.HORIZONTAL,
            variable=self.current_step_size_var, command=self.update_step_size_label, length=200)
        self.step_size_slider.pack(side=tk.LEFT, padx=(0,5))
        self.step_size_slider.set(self.current_step_size_var.get())
        # MODIFIED: Display label for step size
        self.step_size_display_label = ttk.Label(slider_frame, text=f"{self.get_step_size()} steps", width=10)
        self.step_size_display_label.pack(side=tk.LEFT)
        self.update_step_size_label()

        movement_super_frame.columnconfigure(0, weight=1)
        movement_super_frame.columnconfigure(1, weight=1)
        movement_super_frame.columnconfigure(2, weight=1)

        sys_cmd_frame = ttk.LabelFrame(self.main_frame, text="System Commands", padding="5")
        sys_cmd_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        
        # --- NEW: Button to open Degrees Input Window ---
        ttk.Button(sys_cmd_frame, text="Open Positional Control",
                   command=self.open_degrees_input_window).grid(row=0, column=0, padx=5, pady=5)
        # --- END NEW ---

        ttk.Button(sys_cmd_frame, text="Set Verbose ON",
                   command=lambda: self.set_verbose(True)).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(sys_cmd_frame, text="Set Verbose OFF",
                   command=lambda: self.set_verbose(False)).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(sys_cmd_frame, text="Start Test Motors",
                   command=self.show_test_motors_frame).grid(row=0, column=3, padx=5, pady=5)
        sys_cmd_frame.columnconfigure(4, weight=1)

    # MODIFIED: Update step size label to show "steps"
    def update_step_size_label(self, event=None):
        if not hasattr(self, 'step_size_display_label') or not self.step_size_display_label:
            return
        selected_index = int(self.current_step_size_var.get())
        if 0 <= selected_index < len(self.step_sizes):
            current_val = self.step_sizes[selected_index]
            self.step_size_display_label.config(text=f"{current_val} steps") # Changed from °
        else:
            self.step_size_display_label.config(text="N/A")

    def create_output_area(self):
        output_frame = ttk.LabelFrame(self.main_frame, text="Output Log", padding="5")
        output_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.main_frame.grid_rowconfigure(3, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.output_text = tk.Text(output_frame, height=10, width=80) # Increased height
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_frame.grid_rowconfigure(0, weight=1)
        output_frame.grid_columnconfigure(0, weight=1)
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.output_text['yscrollcommand'] = scrollbar.set

    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.port_entry.get()
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)
                self.is_connected = True
                self.status_label.config(text=f"Status: Connected ({port})", foreground="green")
                self.connect_button.config(text="Disconnect")
                self.log_message(f"Connected to {port}")
            except serial.SerialException as e:
                messagebox.showerror("Connection Error", f"Failed to connect to {port}: {str(e)}")
                self.status_label.config(text="Status: Error", foreground="red")
        else:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.status_label.config(text="Status: Disconnected", foreground="black")
            self.connect_button.config(text="Connect")
            self.log_message("Disconnected")

    def send_command(self, command):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode('utf-8'))
                self.log_message(f"Sent: {command}")
            except serial.SerialException as e:
                self.log_message(f"Error sending command: {e}. Lost connection.")
                self.toggle_connection()
            except Exception as e:
                self.log_message(f"Unexpected error sending command: {e}")
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
                except serial.SerialException:
                    if self.is_connected: # only toggle if we thought we were connected
                        self.root.after(0, self.log_message, "Error: Lost connection during serial monitoring.")
                        self.root.after(0, self.toggle_connection) # This will set is_connected to False
                except Exception as e:
                    self.root.after(0, self.log_message, f"Unexpected serial monitoring error: {e}")
            time.sleep(0.1)

    def process_serial_response(self, response):
        if response.startswith("TEST_MOTOR_SELECTED:"):
            name = response.split(":", 1)[1]
            self.selected_motor = name
            self.highlight_selected_motor()
            self.log_test_motors_output(f"Arduino selected motor: {name}")
        else:
            self.log_message(f"Arduino: {response}")
            if hasattr(self, 'test_motors_output_text') and self.test_motors_output_text and \
               self.test_motors_frame and self.test_motors_frame.winfo_ismapped():
               self.log_test_motors_output(f"Arduino: {response}")

    def log_message(self, message):
        if hasattr(self, 'output_text') and self.output_text:
            self.output_text.insert(tk.END, message + "\n")
            self.output_text.see(tk.END)

    # Direct Step Control movement methods now send steps
    def move_pitch_yaw(self, pitch_step_delta, yaw_step_delta):
        # pitch_step_delta and yaw_step_delta are already signed steps from get_step_size()
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
        if steps != 0:
            self.send_command(f"MOVE_WP_REL:{steps}")

    def move_left_jaw(self, step_delta):
        steps = int(step_delta)
        if steps != 0:
            self.send_command(f"MOVE_LJ_REL:{steps}")

    def move_right_jaw(self, step_delta):
        steps = int(step_delta)
        if steps != 0:
            self.send_command(f"MOVE_RJ_REL:{steps}")

    def set_verbose(self, state):
        self.is_verbose = state
        self.send_command(f"SET_VERBOSE:{1 if state else 0}")
        self.log_message(f"Verbose mode {'ON' if state else 'OFF'}")

    def show_test_motors_frame(self):
        if self.test_motors_frame and self.test_motors_frame.winfo_exists():
            self.test_motors_frame.grid()
            self.send_command("START_TEST_MOTORS")
            self.select_test_motor(self.selected_motor)
            return

        self.send_command("START_TEST_MOTORS")
        self.test_motors_frame = ttk.LabelFrame(self.main_frame, text="Test Motors Control", padding="10")
        self.test_motors_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        self.main_frame.grid_rowconfigure(4, weight=1)

        motor_buttons_frame = ttk.Frame(self.test_motors_frame)
        motor_buttons_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N)
        self.motor_buttons = {}
        style = ttk.Style()
        style.configure("SelectedMotor.TButton", background="#0078D7", foreground="white", font=("Segoe UI", 9, "bold"))
        style.map("SelectedMotor.TButton", background=[('active', '#005a9e')], foreground=[('active', 'white')])
        style.configure("TButton", font=("Segoe UI", 9))

        for r, row_names in enumerate(self.motor_names_grouped): # Use grouped names for layout
            for c, name in enumerate(row_names):
                btn = ttk.Button(motor_buttons_frame, text=name, width=6,
                                 command=lambda n=name: self.select_test_motor(n))
                btn.grid(row=r, column=c, pady=3, padx=3)
                self.motor_buttons[name] = btn

        motor_controls_frame = ttk.Frame(self.test_motors_frame)
        motor_controls_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N)

        # --- NEW: Home Link Button ---
        ttk.Button(motor_controls_frame, text="Home Link (Set Home)", width=20, command=self._home_link_command).pack(pady=3, fill=tk.X)
        # --- END NEW ---
        ttk.Button(motor_controls_frame, text="Fine Tension", width=20, command=self.fine_tension_selected_motor).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Coarse Tension", width=20, command=self.coarse_tension_selected_motor).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Detension", width=20, command=self.detension_selected_motor).pack(pady=3, fill=tk.X)

        step_motor_manual_frame = ttk.Frame(motor_controls_frame)
        step_motor_manual_frame.pack(pady=3, fill=tk.X)
        ttk.Button(step_motor_manual_frame, text="Step Motor", width=15, command=self.step_selected_motor_by_amount).pack(side=tk.LEFT,pady=3)
        self.motor_test_step_slider = ttk.Scale(
            step_motor_manual_frame, from_=self.motor_test_step_min, to=self.motor_test_step_max, orient=tk.HORIZONTAL,
            variable=self.motor_test_step_count_var, command=self.update_motor_test_step_label, length=100)
        self.motor_test_step_slider.pack(side=tk.LEFT, padx=5, pady=(3,0), fill=tk.X, expand=True)
        self.motor_test_step_display_label = ttk.Label(step_motor_manual_frame, text=f"{self.motor_test_step_count_var.get()} steps", width=10)
        self.motor_test_step_display_label.pack(side=tk.LEFT, padx=5, pady=(3,0))
        self.update_motor_test_step_label()

        ttk.Button(motor_controls_frame, text="Next Motor", width=20, command=self.next_test_motor).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Exit Test Mode", width=20, command=self.exit_test_motors).pack(pady=3, fill=tk.X)

        test_output_frame = ttk.Frame(self.test_motors_frame)
        test_output_frame.grid(row=0, column=2, padx=10, pady=5, sticky=(tk.N, tk.S, tk.E, tk.W))
        self.test_motors_frame.columnconfigure(2, weight=1)
        self.test_motors_frame.rowconfigure(0, weight=1)
        ttk.Label(test_output_frame, text="Test Motor Log:").grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0,2))
        self.test_motors_output_text = tk.Text(test_output_frame, height=10, width=45) # Adjusted height
        self.test_motors_output_text.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        test_scrollbar = ttk.Scrollbar(test_output_frame, orient=tk.VERTICAL, command=self.test_motors_output_text.yview)
        test_scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        test_output_frame.rowconfigure(1, weight=1)
        test_output_frame.columnconfigure(0, weight=1)
        self.test_motors_output_text['yscrollcommand'] = test_scrollbar.set
        self.highlight_selected_motor()

    def update_motor_test_step_label(self, event=None):
        steps = int(self.motor_test_step_count_var.get())
        self.motor_test_step_display_label.config(text=f"{steps} steps")

    def get_motor_test_step_amount(self):
        try: return int(self.motor_test_step_count_var.get())
        except tk.TclError: return 0 # Default to 0 if error

    def step_selected_motor_by_amount(self):
        steps = self.get_motor_test_step_amount()
        self.send_command(f"STEP_MOTOR_BY:{steps}")
        self.log_test_motors_output(f"Command: STEP_MOTOR_BY:{steps} for {self.selected_motor}")

    def highlight_selected_motor(self):
        if not hasattr(self, 'motor_buttons') or not self.motor_buttons: return
        for name, btn in self.motor_buttons.items():
            if btn.winfo_exists():
                btn.config(style="SelectedMotor.TButton" if name == self.selected_motor else "TButton")

    def select_test_motor(self, name):
        self.selected_motor = name
        self.highlight_selected_motor()
        self.send_command(f"SELECT_MOTOR:{name}")
        self.log_test_motors_output(f"GUI selected motor: {name}")

    def fine_tension_selected_motor(self):
        self.send_command("FINE_TENSION")
        self.log_test_motors_output(f"Command: FINE_TENSION for {self.selected_motor}")

    def coarse_tension_selected_motor(self):
        self.send_command("COARSE_TENSION")
        self.log_test_motors_output(f"Command: COARSE_TENSION for {self.selected_motor}")

    def detension_selected_motor(self):
        self.send_command("DETENSION")
        self.log_test_motors_output(f"Command: DETENSION for {self.selected_motor}")

    def next_test_motor(self):
        self.send_command("NEXT_MOTOR") # Arduino will respond with TEST_MOTOR_SELECTED
        self.log_test_motors_output("Command: NEXT_MOTOR")

    def exit_test_motors(self):
        self.send_command("EXIT_TEST")
        self.log_message("Exited Test Motors mode.")
        if self.test_motors_frame and self.test_motors_frame.winfo_exists():
            self.test_motors_frame.grid_remove()

    def log_test_motors_output(self, message):
        if hasattr(self, 'test_motors_output_text') and self.test_motors_output_text and self.test_motors_output_text.winfo_exists():
            self.test_motors_output_text.insert(tk.END, message + "\n")
            self.test_motors_output_text.see(tk.END)
        else:
            self.log_message(f"(TestMotorLog-Fallback): {message}")

    def get_step_size(self): # Returns actual step value, signed by button command
        try:
            selected_index = int(self.current_step_size_var.get())
            if 0 <= selected_index < len(self.step_sizes):
                return self.step_sizes[selected_index]
        except tk.TclError: pass
        return 10 # Default if error

    # --- NEW: Degrees Input Window and Logic ---
    def open_degrees_input_window(self):
        if self.degrees_input_window and self.degrees_input_window.winfo_exists():
            self.degrees_input_window.lift()
            return

        self.degrees_input_window = Toplevel(self.root)
        self.degrees_input_window.title("Positional Control (Degrees)")
        self.degrees_input_window.geometry("550x350") # Adjusted size
        self.degrees_input_window.resizable(False, False)

        # Make this window transient to the main window
        self.degrees_input_window.transient(self.root)
        self.degrees_input_window.grab_set() # Modal behavior

        main_degrees_frame = ttk.Frame(self.degrees_input_window, padding="10")
        main_degrees_frame.pack(expand=True, fill="both")

        # Input section
        input_frame = ttk.LabelFrame(main_degrees_frame, text="Move by Degrees", padding="10")
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
        cumulative_frame = ttk.LabelFrame(main_degrees_frame, text="Cumulative Joint Position (Degrees from Home)", padding="10")
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

        # Ensure the window closes properly
        self.degrees_input_window.protocol("WM_DELETE_WINDOW", self._on_degrees_window_close)
        self.degrees_input_window.mainloop() # This makes it modal effectively block until closed

    def _on_degrees_window_close(self):
        if self.degrees_input_window:
            self.degrees_input_window.grab_release()
            self.degrees_input_window.destroy()
            self.degrees_input_window = None


    def _move_by_degrees_command(self):
        try:
            ep_deg = float(self.ep_degrees_input_var.get())
            ey_deg = float(self.ey_degrees_input_var.get())
            wp_deg = float(self.wp_degrees_input_var.get())
            lj_deg = float(self.lj_degrees_input_var.get())
            rj_deg = float(self.rj_degrees_input_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.", parent=self.degrees_input_window)
            return

        all_motor_steps = self._calculate_all_motor_steps_from_degrees(ep_deg, ey_deg, wp_deg, lj_deg, rj_deg)
        
        # Send command to Arduino
        command_str = "MOVE_ALL_MOTORS:" + ",".join(map(str, all_motor_steps))
        self.send_command(command_str)

        # Update cumulative displays
        self.cumulative_ep_degrees_var.set(round(self.cumulative_ep_degrees_var.get() + ep_deg, 2))
        self.cumulative_ey_degrees_var.set(round(self.cumulative_ey_degrees_var.get() + ey_deg, 2))
        self.cumulative_wp_degrees_var.set(round(self.cumulative_wp_degrees_var.get() + wp_deg, 2))
        self.cumulative_lj_degrees_var.set(round(self.cumulative_lj_degrees_var.get() + lj_deg, 2))
        self.cumulative_rj_degrees_var.set(round(self.cumulative_rj_degrees_var.get() + rj_deg, 2))

        # Optionally, clear input fields after moving
        # self.ep_degrees_input_var.set("0.0")
        # ... for others

    def _home_link_command(self):
        self.send_command("SET_HOME")
        self._reset_cumulative_degrees_display()
        self.log_message("Home Link activated. SET_HOME sent. Cumulative degrees reset.")
        if self.degrees_input_window and self.degrees_input_window.winfo_exists():
            self.degrees_input_window.lift() # Bring to front if open

    def _reset_cumulative_degrees_display(self):
        self.cumulative_ep_degrees_var.set(0.0)
        self.cumulative_ey_degrees_var.set(0.0)
        self.cumulative_wp_degrees_var.set(0.0)
        self.cumulative_lj_degrees_var.set(0.0)
        self.cumulative_rj_degrees_var.set(0.0)
        if self.degrees_input_window and self.degrees_input_window.winfo_exists():
             messagebox.showinfo("Reset", "Cumulative degree display has been reset to zero.", parent=self.degrees_input_window)


    # --- Placeholder Degree-to-Step Conversion Functions ---
    # YOU NEED TO IMPLEMENT THE ACTUAL LOGIC HERE BASED ON YOUR ROBOT'S KINEMATICS/MECHANICS
    # These functions should return the number of steps for EACH motor involved in the joint movement.
    # The order in the final 10-element array is:
    # [RJR, LJL, LJR, RJL, WPU, WPD, EYR, EYL, EPD, EPU]

    def get_steps_elbow_pitch(self, degrees_ep):
        # EPD (motor_names_flat[8]), EPU (motor_names_flat[9])
        # Example: 1 degree = 100 steps. Positive degrees = pitch UP (EPU positive, EPD negative or other way around)
        # This is a placeholder. Adjust signs and values based on your setup.
        steps = int(degrees_ep * 100) # Placeholder conversion factor
        return {"EPD": -steps, "EPU": steps} # Assuming EPU moves positively for up pitch

    def get_steps_elbow_yaw(self, degrees_ey):
        # EYR (motor_names_flat[6]), EYL (motor_names_flat[7])
        # Example: Positive degrees = yaw RIGHT (EYR positive, EYL negative)
        steps = int(degrees_ey * 100) # Placeholder
        return {"EYR": steps, "EYL": -steps} # Assuming EYR moves positively for right yaw

    def get_steps_wrist_pitch(self, degrees_wp):
        # WPU (motor_names_flat[4]), WPD (motor_names_flat[5])
        # Example: Positive degrees = pitch UP (WPU positive, WPD negative)
        steps = int(degrees_wp * 80) # Placeholder
        return {"WPU": steps, "WPD": -steps} # Assuming WPU moves positively for up pitch

    def get_steps_left_jaw(self, degrees_lj):
        # LJR (motor_names_flat[2]), RJL (motor_names_flat[3]) - Note: motor_names_flat might need re-evaluation for L/R jaw
        # For simplicity, let's assume LJR and RJL form the "Left Jaw" pair.
        # This mapping needs to be precise for your robot.
        # Example: Positive degrees = open jaw
        steps = int(degrees_lj * 50) # Placeholder
        return {"LJR": steps, "RJL": -steps} # Placeholder: LJR opens, RJL helps or closes

    def get_steps_right_jaw(self, degrees_rj):
        # RJR (motor_names_flat[0]), LJL (motor_names_flat[1])
        # Example: Positive degrees = open jaw
        steps = int(degrees_rj * 50) # Placeholder
        return {"RJR": steps, "LJL": -steps} # Placeholder: RJR opens, LJL helps or closes

    def _calculate_all_motor_steps_from_degrees(self, ep_deg, ey_deg, wp_deg, lj_deg, rj_deg):
        # Initialize a 10-element list for all motor steps
        # Order: [RJR, LJL, LJR, RJL, WPU, WPD, EYR, EYL, EPD, EPU]
        motor_steps = [0] * 10

        # Elbow Pitch
        ep_motor_steps = self.get_steps_elbow_pitch(ep_deg)
        motor_steps[8] = ep_motor_steps.get("EPD", 0) # EPD
        motor_steps[9] = ep_motor_steps.get("EPU", 0) # EPU

        # Elbow Yaw
        ey_motor_steps = self.get_steps_elbow_yaw(ey_deg)
        motor_steps[6] = ey_motor_steps.get("EYR", 0) # EYR
        motor_steps[7] = ey_motor_steps.get("EYL", 0) # EYL

        # Wrist Pitch
        wp_motor_steps = self.get_steps_wrist_pitch(wp_deg)
        motor_steps[4] = wp_motor_steps.get("WPU", 0) # WPU
        motor_steps[5] = wp_motor_steps.get("WPD", 0) # WPD

        # Left Jaw
        lj_motor_steps = self.get_steps_left_jaw(lj_deg)
        motor_steps[2] = lj_motor_steps.get("LJR", 0) # LJR
        motor_steps[3] = lj_motor_steps.get("RJL", 0) # RJL (part of Left Jaw pair in this example)

        # Right Jaw
        rj_motor_steps = self.get_steps_right_jaw(rj_deg)
        motor_steps[0] = rj_motor_steps.get("RJR", 0) # RJR
        motor_steps[1] = rj_motor_steps.get("LJL", 0) # LJL (part of Right Jaw pair in this example)

        self.log_message(f"Calculated steps: {motor_steps}")
        return motor_steps

    # --- END NEW ---

def main():
    root = tk.Tk()
    style = ttk.Style(root)
    available_themes = style.theme_names()
    # Prefer modern themes
    if "clam" in available_themes: style.theme_use("clam")
    elif "vista" in available_themes: style.theme_use("vista") # Windows
    elif "aqua" in available_themes: style.theme_use("aqua") # macOS
    # Add other preferred themes if needed, e.g., 'alt', 'default'

    app = ElbowSimulatorGUI(root)
    # Ensure proper cleanup on close
    def on_closing():
        if app.is_connected:
            app.log_message("Closing connection before exit...")
            if app.serial_port and app.serial_port.is_open:
                app.send_command("EXIT_TEST") # Try to gracefully exit test mode if active
                time.sleep(0.1)
                app.serial_port.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()