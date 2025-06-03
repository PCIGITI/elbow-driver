import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import config # For constants, MotorIndex
from config import MotorIndex
from test_motors_gui import TestMotorsWindow # The Toplevel window
import q1_pl, q2_pl, q3_pl, q4_pl # Joint processors for step calculations

# motor_control_logic.py will be instantiated in main.py and passed here
# serial_handler.py will be instantiated in main.py and passed here

class ElbowSimulatorGUI:
    def __init__(self, root, serial_handler):
        self.root = root
        self.serial_handler = serial_handler

        self.root.title(config.APP_TITLE)
        self.root.geometry(config.MAIN_WINDOW_GEOMETRY)

        # Pass callbacks to serial_handler 
        self.serial_handler.set_callbacks(
            data_callback=self._handle_serial_data,
            status_callback=self._update_connection_status_display,
            error_callback=self._handle_serial_error
        )
        self.is_verbose_arduino_side = False # To track Arduino's verbose state
        self.test_motors_window_instance = None
        self.selected_motor_for_test_mode = config.MOTOR_NAMES_FLAT[8] # Default to "EPD"

        # --- Tkinter Variables ---
        
        # Step Control
        self.current_step_size_var = tk.IntVar(value=config.DEFAULT_STEP_SIZE_INDEX)
        # Positional Control Inputs
        self.ep_degrees_input_var = tk.StringVar(value="0.0")
        self.ey_degrees_input_var = tk.StringVar(value="0.0")
        self.wp_degrees_input_var = tk.StringVar(value="0.0")
        self.lj_degrees_input_var = tk.StringVar(value="0.0")
        self.rj_degrees_input_var = tk.StringVar(value="0.0")
        #self.roll_degrees_input_var = tk.StringVar(value="0.0")


        # Positional Control Cumulative Display
        self.cumulative_ep_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_ey_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_wp_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_lj_degrees_var = tk.DoubleVar(value=0.0)
        self.cumulative_rj_degrees_var = tk.DoubleVar(value=0.0)
        #self.cumulative_roll_degrees_var = tk.DoubleVar(value=0.0)

        self._setup_main_layout()
        self._create_widgets()
        self._create_output_area() # Ensure output_text is created before logging

        self.log_message(f"{config.APP_TITLE} initialized.")


    def _setup_main_layout(self):
        self.canvas = tk.Canvas(self.root, borderwidth=0, background="#f0f0f0")
        self.v_scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.v_scrollbar.set)
        self.v_scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.container_frame = ttk.Frame(self.canvas)
        self.main_frame_id = self.canvas.create_window((0, 0), window=self.container_frame, anchor="nw")

        self.main_content_frame = ttk.Frame(self.container_frame, padding="10") # Renamed for clarity
        self.main_content_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.container_frame.columnconfigure(0, weight=1)

        self.container_frame.bind("<Configure>", self._on_frame_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

    def _on_frame_configure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.canvas.itemconfig(self.main_frame_id, width=event.width)

    def _create_widgets(self):
        # Row 0: Connection
        conn_frame = ttk.LabelFrame(self.main_content_frame, text="Arduino Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(conn_frame, width=10)
        self.port_entry.insert(0, config.DEFAULT_SERIAL_PORT)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection_action)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected")
        self.status_label.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        conn_frame.columnconfigure(3, weight=1)

        # Row 1: Direct Step Control
        movement_super_frame = ttk.LabelFrame(self.main_content_frame, text="Direct Step Control", padding="5")
        movement_super_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)

        # Elbow D-Pad
        elbow_ctrl_frame = ttk.Frame(movement_super_frame, padding="5")
        elbow_ctrl_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N)
        ttk.Label(elbow_ctrl_frame, text="Elbow Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0,5))
        # ... (D-pad buttons, commands call self._direct_move_pitch_yaw_action)
        ttk.Button(elbow_ctrl_frame, text="↖", width=3, command=lambda: self._direct_move_pitch_yaw_action(self._get_current_step_size(), -self._get_current_step_size())).grid(row=1, column=0)
        ttk.Button(elbow_ctrl_frame, text="↑", width=3, command=lambda: self._direct_move_pitch_yaw_action(self._get_current_step_size(), 0)).grid(row=1, column=1)
        ttk.Button(elbow_ctrl_frame, text="↗", width=3, command=lambda: self._direct_move_pitch_yaw_action(self._get_current_step_size(), self._get_current_step_size())).grid(row=1, column=2)
        ttk.Button(elbow_ctrl_frame, text="←", width=3, command=lambda: self._direct_move_pitch_yaw_action(0, -self._get_current_step_size())).grid(row=2, column=0)
        tk.Label(elbow_ctrl_frame, text=" ").grid(row=2, column=1) # Spacer
        ttk.Button(elbow_ctrl_frame, text="→", width=3, command=lambda: self._direct_move_pitch_yaw_action(0, self._get_current_step_size())).grid(row=2, column=2)
        ttk.Button(elbow_ctrl_frame, text="↙", width=3, command=lambda: self._direct_move_pitch_yaw_action(-self._get_current_step_size(), -self._get_current_step_size())).grid(row=3, column=0)
        ttk.Button(elbow_ctrl_frame, text="↓", width=3, command=lambda: self._direct_move_pitch_yaw_action(-self._get_current_step_size(), 0)).grid(row=3, column=1)
        ttk.Button(elbow_ctrl_frame, text="↘", width=3, command=lambda: self._direct_move_pitch_yaw_action(-self._get_current_step_size(), self._get_current_step_size())).grid(row=3, column=2)

        # Wrist Pitch Control
        wrist_ctrl_frame = ttk.Frame(movement_super_frame, padding="5")
        wrist_ctrl_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N)
        ttk.Label(wrist_ctrl_frame, text="Wrist Pitch Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, pady=(0,5))
        ttk.Button(wrist_ctrl_frame, text="↑", width=4, command=lambda: self._direct_move_wrist_pitch_action(self._get_current_step_size())).grid(row=1, column=0)
        ttk.Button(wrist_ctrl_frame, text="↓", width=4, command=lambda: self._direct_move_wrist_pitch_action(-self._get_current_step_size())).grid(row=2, column=0)

        # Jaw Control
        jaw_ctrl_frame = ttk.Frame(movement_super_frame, padding="5")
        jaw_ctrl_frame.grid(row=0, column=2, padx=10, pady=5, sticky=tk.N)
        ttk.Label(jaw_ctrl_frame, text="Jaw Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0,5))
        ttk.Label(jaw_ctrl_frame, text="Left Jaw:").grid(row=1, column=0, sticky=tk.E)
        ttk.Button(jaw_ctrl_frame, text="←", width=3, command=lambda: self._direct_move_left_jaw_action(-self._get_current_step_size())).grid(row=1, column=1)
        ttk.Button(jaw_ctrl_frame, text="→", width=3, command=lambda: self._direct_move_left_jaw_action(self._get_current_step_size())).grid(row=1, column=2)
        ttk.Label(jaw_ctrl_frame, text="Right Jaw:").grid(row=2, column=0, sticky=tk.E)
        ttk.Button(jaw_ctrl_frame, text="←", width=3, command=lambda: self._direct_move_right_jaw_action(-self._get_current_step_size())).grid(row=2, column=1)
        ttk.Button(jaw_ctrl_frame, text="→", width=3, command=lambda: self._direct_move_right_jaw_action(self._get_current_step_size())).grid(row=2, column=2)

        # Step Size Slider
        slider_frame = ttk.Frame(movement_super_frame)
        slider_frame.grid(row=1, column=0, columnspan=3, pady=(10,5))
        ttk.Label(slider_frame, text="Movement Steps:").pack(side=tk.LEFT, padx=(0,5))
        self.step_size_slider = ttk.Scale(
            slider_frame, from_=0, to=len(config.STEP_SIZES) - 1, orient=tk.HORIZONTAL,
            variable=self.current_step_size_var, command=self._update_step_size_display_label, length=200)
        self.step_size_slider.pack(side=tk.LEFT, padx=(0,5))
        self.step_size_slider.set(self.current_step_size_var.get())
        self.step_size_display_label = ttk.Label(slider_frame, text="", width=10) # Text set by _update_step_size_display_label
        self.step_size_display_label.pack(side=tk.LEFT)
        self._update_step_size_display_label() # Initialize

        movement_super_frame.columnconfigure(0, weight=1); movement_super_frame.columnconfigure(1, weight=1); movement_super_frame.columnconfigure(2, weight=1)


        # Row 2: System Commands
        sys_cmd_frame = ttk.LabelFrame(self.main_content_frame, text="System Commands", padding="5")
        sys_cmd_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)
        # Toggle Verbose button instead of two separate ones
        self.verbose_button = ttk.Button(sys_cmd_frame, text="Toggle Verbose (OFF)", command=self._toggle_verbose_action)
        self.verbose_button.grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(sys_cmd_frame, text="Start Test Motors", command=self._open_test_motors_window_action).grid(row=0, column=1, padx=5, pady=5)
        sys_cmd_frame.columnconfigure(2, weight=1) # Adjust if more buttons

        # Row 3: Inline Positional Control Frame
        self._create_positional_control_frame_widgets(self.main_content_frame)

    def _create_positional_control_frame_widgets(self, parent_frame):
        positional_super_frame = ttk.LabelFrame(parent_frame, text="Positional Control (Degrees)", padding="10")
        positional_super_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        parent_frame.grid_rowconfigure(3, weight=0) # Not expanding much

        # Input section
        input_frame = ttk.LabelFrame(positional_super_frame, text="Move by Degrees", padding="10")
        input_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ns")
        # EP, EY, WP, LJ, RJ Entries and Labels...
        ttk.Label(input_frame, text="Elbow Pitch (°):").grid(row=0, column=0, sticky="w", pady=2)
        self.ep_degrees_entry = ttk.Entry(input_frame, textvariable=self.ep_degrees_input_var, width=10)
        self.ep_degrees_entry.grid(row=0, column=1, pady=2)
        # ... ( EY, WP, LJ, RJ entries similar to EP ) ...
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

        ttk.Button(input_frame, text="Move Joints by Degrees", command=self._move_by_degrees_action, width=25).grid(row=5, column=0, columnspan=2, pady=10)

        # Cumulative display section
        cumulative_frame = ttk.LabelFrame(positional_super_frame, text="Cumulative Joint Position (Degrees from Home)", padding="10")
        cumulative_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ns")
        # EP, EY, WP, LJ, RJ Labels for cumulative display...
        ttk.Label(cumulative_frame, text="Elbow Pitch:").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ep_degrees_var, width=8, anchor="e").grid(row=0, column=1, sticky="e", pady=2)
        ttk.Label(cumulative_frame, text="°").grid(row=0, column=2, sticky="w", pady=2)
        # ... (EY, WP, LJ, RJ cumulative labels similar to EP) ...
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
        
        ttk.Button(cumulative_frame, text="Reset Cumulative Display", command=self._reset_cumulative_degrees_display_action, width=25).grid(row=5, column=0, columnspan=3, pady=10)
        
        positional_super_frame.columnconfigure(0, weight=1)
        positional_super_frame.columnconfigure(1, weight=1)

    def _create_output_area(self):
        output_frame = ttk.LabelFrame(self.main_content_frame, text="Output Log", padding="5")
        output_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.main_content_frame.grid_rowconfigure(4, weight=1)
        self.main_content_frame.grid_columnconfigure(0, weight=1) # Ensure main_content_frame's first col expands

        self.output_text = tk.Text(output_frame, height=10, width=80) # Width might be less critical due to expansion
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        output_frame.grid_rowconfigure(0, weight=1)
        output_frame.grid_columnconfigure(0, weight=1)
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.output_text['yscrollcommand'] = scrollbar.set

    def log_message(self, message):
        if hasattr(self, 'output_text') and self.output_text:
            self.output_text.insert(tk.END, str(message) + "\n")
            self.output_text.see(tk.END)
        else:
            print(f"LOG (output_text not ready): {message}") # Fallback if called too early

    # --- Serial Communication Callbacks ---
    def _update_connection_status_display(self, message, color, connected):
        self.status_label.config(text=f"Status: {message}", foreground=color)
        self.connect_button.config(text="Disconnect" if connected else "Connect")
        self.log_message(message) # Also log status changes to the main log

    def _handle_serial_data(self, response_data, data_type="received"): # data_type can be "sent" or "received"
        # This is where you process incoming messages from Arduino
        self.log_message(f"Arduino: {response_data}") # Generic log for now

        if response_data.startswith("TEST_MOTOR_SELECTED:"):
            name = response_data.split(":", 1)[1]
            self.selected_motor_for_test_mode = name # Update main GUI's tracking variable
            if self.test_motors_window_instance and self.test_motors_window_instance.winfo_exists():
                self.test_motors_window_instance.update_selected_motor_from_arduino(name)
            # Log in test window as well if open
            if self.test_motors_window_instance and self.test_motors_window_instance.winfo_exists():
                 self.test_motors_window_instance.log_to_test_output(f"Arduino selected motor: {name}")
            else: # Log to main if test window isn't open (e.g. after NEXT_MOTOR then EXIT_TEST quickly)
                 self.log_message(f"(TestLog Echo) Arduino selected motor: {name}")


        elif response_data.startswith("VERBOSE_STATE:"):
            state = response_data.split(":")[1].strip()
            self.is_verbose_arduino_side = (state == "1")
            self.verbose_button.config(text=f"Toggle Verbose ({'ON' if self.is_verbose_arduino_side else 'OFF'})")
            self.log_message(f"Verbose mode {'ON' if self.is_verbose_arduino_side else 'OFF'}")
        else:
            # If test motors window is open, echo general Arduino messages there too
            if self.test_motors_window_instance and self.test_motors_window_instance.winfo_exists():
                if hasattr(self.test_motors_window_instance, 'log_to_test_output'):
                     self.test_motors_window_instance.log_to_test_output(f"Arduino: {response_data}")

    def _handle_serial_error(self, error_message):
        messagebox.showerror("Serial Error", error_message, parent=self.root)
        self.log_message(f"ERROR: {error_message}")

    # --- Button Action Methods (GUI Event Handlers) ---
    def _toggle_connection_action(self):
        if not self.serial_handler.is_connected:
            port = self.port_entry.get()
            self.serial_handler.connect(port)
        else:
            self.serial_handler.disconnect()

    def _get_current_step_size(self):
        try:
            selected_index = int(self.current_step_size_var.get())
            if 0 <= selected_index < len(config.STEP_SIZES):
                return config.STEP_SIZES[selected_index]
        except tk.TclError: pass # If var not fully initialized
        return config.STEP_SIZES[config.DEFAULT_STEP_SIZE_INDEX] # Default

    def _update_step_size_display_label(self, event=None):
        if not hasattr(self, 'step_size_display_label') or not self.step_size_display_label: return
        current_val = self._get_current_step_size()
        self.step_size_display_label.config(text=f"{current_val} steps")

    def _direct_move_pitch_yaw_action(self, pitch_delta, yaw_delta):
        if pitch_delta == 0 and yaw_delta == 0: return
        
        motor_steps = [0] * len(MotorIndex)

        motor_steps[MotorIndex.EPD] = pitch_delta
        motor_steps[MotorIndex.EPU] = -pitch_delta
        motor_steps[MotorIndex.EYR] = yaw_delta
        motor_steps[MotorIndex.EYL] = -yaw_delta

        steps_str = ",".join(map(str, motor_steps))
        cmd = f"MOVE_ALL_MOTORS:{steps_str}"
        self.serial_handler.send_command(cmd)
        self.log_message(f"Sent Direct Elbow: {cmd}")

    def _direct_move_wrist_pitch_action(self, step_delta):
        if step_delta == 0: return
        motor_steps = [0] * len(MotorIndex)

        motor_steps[MotorIndex.WPD] = step_delta
        motor_steps[MotorIndex.WPU] = -step_delta

        steps_str = ",".join(map(str, motor_steps))
        cmd = f"MOVE_ALL_MOTORS:{steps_str}"
        self.serial_handler.send_command(cmd)
        self.log_message(f"Sent Direct Wrist Pitch: {cmd}")

    def _direct_move_left_jaw_action(self, step_delta):
        if step_delta == 0: return
        motor_steps = [0] * len(MotorIndex)

        motor_steps[MotorIndex.LJR] = step_delta
        motor_steps[MotorIndex.LJL] = -step_delta

        steps_str = ",".join(map(str, motor_steps))
        cmd = f"MOVE_ALL_MOTORS:{steps_str}"
        self.serial_handler.send_command(cmd)
        self.log_message(f"Sent Direct Left Jaw: {cmd}")

    def _direct_move_right_jaw_action(self, step_delta):
        if step_delta == 0: return

        motor_steps = [0] * len(MotorIndex)

        motor_steps[MotorIndex.RJR] = step_delta
        motor_steps[MotorIndex.RJL] = -step_delta
        
        steps_str = ",".join(map(str, motor_steps))
        cmd = f"MOVE_ALL_MOTORS:{steps_str}"
        self.serial_handler.send_command(cmd)
        self.log_message(f"Sent Direct Right Jaw: {cmd}")


    def _toggle_verbose_action(self):
        self.serial_handler.send_command("TOGGLE_VERBOSE")
        self.log_message("Sent: TOGGLE_VERBOSE")
        # Actual button text updates on Arduino response (VERBOSE_STATE:)

    def _open_test_motors_window_action(self):
        if self.test_motors_window_instance and self.test_motors_window_instance.winfo_exists():
            self.test_motors_window_instance.lift()
            return
        self.test_motors_window_instance = TestMotorsWindow(
            self.root,
            self.serial_handler,
            self.selected_motor_for_test_mode,
            self.log_message # Pass main log for messages like "Exited test mode"
        )

    def on_test_motors_window_closed(self):
        self.test_motors_window_instance = None
        self.log_message("Test Motors window closed by user.")

    def _move_by_degrees_action(self):
        try:
            joint_degree_deltas = {
                "EP": float(self.ep_degrees_input_var.get()),
                "EY": float(self.ey_degrees_input_var.get()),
                "WP": float(self.wp_degrees_input_var.get()),
                "LJ": float(self.lj_degrees_input_var.get()),
                "RJ": float(self.rj_degrees_input_var.get()),
            }
        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.", parent=self.root)
            return

        ep_deg = self.cumulative_ep_degrees_var.get()
        ey_deg = self.cumulative_ey_degrees_var.get()   
        wp_deg = self.cumulative_wp_degrees_var.get()
        lj_deg = self.cumulative_lj_degrees_var.get()
        rj_deg = self.cumulative_rj_degrees_var.get()

        ep_delta = float(self.ep_degrees_input_var.get())
        ey_delta = float(self.ey_degrees_input_var.get())
        wp_delta = float(self.wp_degrees_input_var.get())
        lj_delta = float(self.lj_degrees_input_var.get())
        rj_delta = float(self.rj_degrees_input_var.get())

        
        joint_processors = {
            "EP": (ep_deg, ep_delta, q1_pl.get_steps),  # Elbow Pitch
            "EY": (ey_deg, ey_delta, q2_pl.get_steps),  # Elbow Yaw
            "WP": (wp_deg, wp_delta, q3_pl.get_steps),  # Wrist Pitch
            "LJ": (lj_deg, lj_delta, q4_pl.get_steps_L),  # Left Jaw
            "RJ": (rj_deg, rj_delta, q4_pl.get_steps_R),   # Right Jaw
        }

        total_motor_steps = [0] * len(MotorIndex)  # Initialize total steps for all motors

        for joint_name, (curr_theta, delta_theta, get_steps_function) in joint_processors.items():
            if delta_theta != 0:
                try:
                    self.log_message(f"Calculating steps for {joint_name} with delta: {delta_theta}")
                    joint_specific_motor_steps = get_steps_function(curr_theta, delta_theta)
                    for idx in MotorIndex:
                        total_motor_steps[idx] += joint_specific_motor_steps[idx]
                    self.log_message(f"Steps from {joint_name}: {joint_specific_motor_steps}")
                except Exception as e:
                    self.log_message(f"Error calling {get_steps_function.__name__} for {joint_name}: {e}")
        
        final_integer_steps = [int(round(s)) for s in total_motor_steps]
        self.log_message(f"Final calculated steps for all motors: {final_integer_steps}")
               
        steps_str = ",".join(map(str, final_integer_steps))
        cmd = f"MOVE_ALL_MOTORS:{steps_str}"

        if self.serial_handler.send_command(cmd):
            self.log_message(f"Sent Positional: {cmd}")
            # Update cumulative degrees display AFTER command is successfully sent
            self.cumulative_ep_degrees_var.set(round(self.cumulative_ep_degrees_var.get() + joint_degree_deltas["EP"], 2))
            self.cumulative_ey_degrees_var.set(round(self.cumulative_ey_degrees_var.get() + joint_degree_deltas["EY"], 2))
            self.cumulative_wp_degrees_var.set(round(self.cumulative_wp_degrees_var.get() + joint_degree_deltas["WP"], 2))
            self.cumulative_lj_degrees_var.set(round(self.cumulative_lj_degrees_var.get() + joint_degree_deltas["LJ"], 2))
            self.cumulative_rj_degrees_var.set(round(self.cumulative_rj_degrees_var.get() + joint_degree_deltas["RJ"], 2))
        else:
            self.log_message("Failed to send positional command.")


    def _reset_cumulative_degrees_display_action(self, from_test_mode=False):
        self.cumulative_ep_degrees_var.set(90.0)
        self.cumulative_ey_degrees_var.set(90.0)
        self.cumulative_wp_degrees_var.set(90.0)
        self.cumulative_lj_degrees_var.set(90.0)
        self.cumulative_rj_degrees_var.set(90.0)
        if not from_test_mode: # Don't show popup if reset was triggered by "Set Home" in test window
            messagebox.showinfo("Reset", "Cumulative degree display has been reset to zero.", parent=self.root)
        self.log_message("Cumulative degree display reset.")

    def _reset_cumulative_degrees_display_from_test_mode(self):
        # Called by TestMotorsWindow after "SET_HOME"
        self._reset_cumulative_degrees_display_action(from_test_mode=True)
        self.log_message("Cumulative degrees reset due to 'Set Home' in Test Motors mode.")


    def cleanup_on_exit(self):
        self.log_message("Application exiting. Cleaning up...")
        if self.test_motors_window_instance and self.test_motors_window_instance.winfo_exists():
            # Try to gracefully exit test mode on Arduino if window is still open
            self.serial_handler.send_command("EXIT_TEST")
            self.log_message("Sent EXIT_TEST on close.")
            # self.test_motors_window_instance.destroy() # It will be destroyed with root
        self.serial_handler.cleanup()
        self.log_message("Serial handler cleaned up.")