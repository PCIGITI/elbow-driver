import tkinter as tk
from tkinter import ttk, messagebox
import math

import config # For constants, MotorIndex
from config import MotorIndex #
import q1_pl, q2_pl, q3_pl, q4_pl # Joint processors for step calculations


class ElbowSimulatorGUI:
    def __init__(self, root, serial_handler): #
        self.root = root #
        self.serial_handler = serial_handler #

        self.root.title(config.APP_TITLE) #
        self.root.geometry(config.MAIN_WINDOW_GEOMETRY) #

        self.serial_handler.set_callbacks( #
            data_callback=self._handle_serial_data, #
            status_callback=self._update_connection_status_display, #
            error_callback=self._handle_serial_error #
        )
        self.is_verbose_arduino_side = False #

        # --- Tkinter Variables ---
        self.current_step_size_var = tk.IntVar(value=config.DEFAULT_STEP_SIZE_INDEX) #
        
        self.d_pad_degree_mode_var = tk.BooleanVar(value=False) # Default to Degree mode for D-Pads
        
        # Moved from _create_positional_control_frame_widgets
        self.wrist_yaw_mode_var = tk.BooleanVar(value=False)  # False = Jaws Mode, True = Wrist Yaw Mode

        # Positional Control Inputs (for the dedicated degree input fields)
        self.ep_degrees_input_var = tk.StringVar(value="0.0") #
        self.ey_degrees_input_var = tk.StringVar(value="0.0") #
        self.wp_degrees_input_var = tk.StringVar(value="0.0") #
        self.lj_degrees_input_var = tk.StringVar(value="0.0") #
        self.rj_degrees_input_var = tk.StringVar(value="0.0") #
        self.wrist_yaw_degrees_var = tk.StringVar(value="0.0") # For Wrist Yaw mode

        # Positional Control Cumulative Display
        self.cumulative_ep_degrees_var = tk.DoubleVar(value=90.0) #
        self.cumulative_ey_degrees_var = tk.DoubleVar(value=90.0) #
        self.cumulative_wp_degrees_var = tk.DoubleVar(value=90.0) #
        self.cumulative_lj_degrees_var = tk.DoubleVar(value=90.0) #
        self.cumulative_rj_degrees_var = tk.DoubleVar(value=90.0) #

        #holds the latest direction values for each motor pair, 0 for true neutral, 1 for ccw, -1 for cw
        self.latest_dir = {
            "EP": 0,
            "EY": 0,
            "WP": 0,
            "LJ": 0,
            "RJ": 0,
        }

        self._setup_main_layout() #
        self._create_widgets() #
        self._create_output_area() # Create the serial output log box
        self._update_d_pad_mode_and_slider_label() # Initialize D-pad mode display
        # self._update_jaw_mode_ui() # Called at the end of _create_positional_control_frame_widgets

        self.log_message(f"{config.APP_TITLE} initialized.") #

    def _setup_main_layout(self): #
        self.canvas = tk.Canvas(self.root, borderwidth=0, background="#f0f0f0") #
        self.v_scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview) #
        self.canvas.configure(yscrollcommand=self.v_scrollbar.set) #
        self.v_scrollbar.pack(side="right", fill="y") #
        self.canvas.pack(side="left", fill="both", expand=True) #

        self.container_frame = ttk.Frame(self.canvas) #
        self.main_frame_id = self.canvas.create_window((0, 0), window=self.container_frame, anchor="nw") #

        self.main_content_frame = ttk.Frame(self.container_frame, padding="10") #
        self.main_content_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S)) #
        self.container_frame.columnconfigure(0, weight=1) #

        self.container_frame.bind("<Configure>", self._on_frame_configure) #
        self.canvas.bind("<Configure>", self._on_canvas_configure) #

    def _on_frame_configure(self, event): #
        self.canvas.configure(scrollregion=self.canvas.bbox("all")) #

    def _on_canvas_configure(self, event): #
        self.canvas.itemconfig(self.main_frame_id, width=event.width) #

    def _create_widgets(self): #
        # Row 0: Connection
        conn_frame = ttk.LabelFrame(self.main_content_frame, text="Arduino Connection", padding="5") #
        conn_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5) #
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5) #
        self.port_entry = ttk.Entry(conn_frame, width=10) #
        self.port_entry.insert(0, config.DEFAULT_SERIAL_PORT) #
        self.port_entry.grid(row=0, column=1, padx=5, pady=5) #
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection_action) #
        self.connect_button.grid(row=0, column=2, padx=5, pady=5) #
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected") #
        self.status_label.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W) #
        conn_frame.columnconfigure(3, weight=1) #

        # Row 1: D-Pad Controls
        d_pad_super_frame = ttk.LabelFrame(self.main_content_frame, text="D-Pad Controls", padding="5") #
        d_pad_super_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5) #

        d_pads_container = ttk.Frame(d_pad_super_frame) #
        d_pads_container.grid(row=0, column=0, columnspan=3) #

        elbow_ctrl_frame = ttk.Frame(d_pads_container, padding="5") #
        elbow_ctrl_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N) #
        ttk.Label(elbow_ctrl_frame, text="Elbow Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0,5)) #
        ttk.Button(elbow_ctrl_frame, text="↖", width=3, command=lambda: self._d_pad_elbow_action(-self._get_slider_value(), -self._get_slider_value())).grid(row=1, column=0) # # sign changed based on image
        ttk.Button(elbow_ctrl_frame, text="↑", width=3, command=lambda: self._d_pad_elbow_action(-self._get_slider_value(), 0)).grid(row=1, column=1) # # sign changed based on image
        ttk.Button(elbow_ctrl_frame, text="↗", width=3, command=lambda: self._d_pad_elbow_action(-self._get_slider_value(), self._get_slider_value())).grid(row=1, column=2) # # sign changed based on image
        ttk.Button(elbow_ctrl_frame, text="←", width=3, command=lambda: self._d_pad_elbow_action(0, -self._get_slider_value())).grid(row=2, column=0) # # sign changed based on image
        tk.Label(elbow_ctrl_frame, text=" ").grid(row=2, column=1) #
        ttk.Button(elbow_ctrl_frame, text="→", width=3, command=lambda: self._d_pad_elbow_action(0, self._get_slider_value())).grid(row=2, column=2) # # sign changed based on image
        ttk.Button(elbow_ctrl_frame, text="↙", width=3, command=lambda: self._d_pad_elbow_action(self._get_slider_value(), -self._get_slider_value())).grid(row=3, column=0) # # sign changed based on image
        ttk.Button(elbow_ctrl_frame, text="↓", width=3, command=lambda: self._d_pad_elbow_action(self._get_slider_value(), 0)).grid(row=3, column=1) # # sign changed based on image
        ttk.Button(elbow_ctrl_frame, text="↘", width=3, command=lambda: self._d_pad_elbow_action(self._get_slider_value(), self._get_slider_value())).grid(row=3, column=2) # # sign changed based on image
        
        wrist_ctrl_frame = ttk.Frame(d_pads_container, padding="5") #
        wrist_ctrl_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N) #
        ttk.Label(wrist_ctrl_frame, text="Wrist Pitch Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, pady=(0,5)) #
        ttk.Button(wrist_ctrl_frame, text="↑", width=4, command=lambda: self._d_pad_wrist_action(-self._get_slider_value())).grid(row=1, column=0) # # sign changed based on image
        ttk.Button(wrist_ctrl_frame, text="↓", width=4, command=lambda: self._d_pad_wrist_action(self._get_slider_value())).grid(row=2, column=0) # # sign changed based on image

        jaw_ctrl_frame = ttk.Frame(d_pads_container, padding="5") #
        jaw_ctrl_frame.grid(row=0, column=2, padx=10, pady=5, sticky=tk.N) #
        ttk.Label(jaw_ctrl_frame, text="Jaw Ctrl", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0,5)) #
        ttk.Label(jaw_ctrl_frame, text="Left Jaw:").grid(row=1, column=0, sticky=tk.E) #

        jaw_dir = 1
        ttk.Button(jaw_ctrl_frame, text="←", width=3, command=lambda: self._d_pad_left_jaw_action(jaw_dir*self._get_slider_value())).grid(row=1, column=1) #
        ttk.Button(jaw_ctrl_frame, text="→", width=3, command=lambda: self._d_pad_left_jaw_action(-1*jaw_dir*self._get_slider_value())).grid(row=1, column=2) #
        ttk.Label(jaw_ctrl_frame, text="Right Jaw:").grid(row=2, column=0, sticky=tk.E) #
        ttk.Button(jaw_ctrl_frame, text="←", width=3, command=lambda: self._d_pad_right_jaw_action(jaw_dir*self._get_slider_value())).grid(row=2, column=1) # # sign changed based on image
        ttk.Button(jaw_ctrl_frame, text="→", width=3, command=lambda: self._d_pad_right_jaw_action(-1*jaw_dir*self._get_slider_value())).grid(row=2, column=2) # # sign changed based on image
        
        d_pads_container.columnconfigure(0, weight=1) #
        d_pads_container.columnconfigure(1, weight=1) #
        d_pads_container.columnconfigure(2, weight=1) #

        d_pad_mode_slider_frame = ttk.Frame(d_pad_super_frame) #
        d_pad_mode_slider_frame.grid(row=1, column=0, columnspan=3, pady=(10,5)) #

        self.d_pad_mode_button = ttk.Checkbutton( #
            d_pad_mode_slider_frame, #
            text="D-Pad Mode: Degrees", # Initial text, will be updated
            variable=self.d_pad_degree_mode_var, #
            command=self._update_d_pad_mode_and_slider_label, #
            style="Toolbutton" # Makes it look more like a toggle button
        )
        self.d_pad_mode_button.pack(side=tk.LEFT, padx=(0,10)) #
        
        ttk.Label(d_pad_mode_slider_frame, text="Increment:").pack(side=tk.LEFT, padx=(0,5)) #
        self.step_size_slider = ttk.Scale( #
            d_pad_mode_slider_frame, from_=0, to=len(config.STEP_SIZES) - 1, orient=tk.HORIZONTAL, #
            variable=self.current_step_size_var, command=self._update_d_pad_mode_and_slider_label, length=150) #
        self.step_size_slider.pack(side=tk.LEFT, padx=(0,5)) #
        self.step_size_slider.set(self.current_step_size_var.get()) #
        
        self.d_pad_increment_display_label = ttk.Label(d_pad_mode_slider_frame, text="", width=12) # Renamed for clarity
        self.d_pad_increment_display_label.pack(side=tk.LEFT) #

        # Row 2: System Commands
        sys_cmd_frame = ttk.LabelFrame(self.main_content_frame, text="System Commands", padding="5") #
        sys_cmd_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10) #
        
        self.verbose_button = ttk.Button(sys_cmd_frame, text="Toggle Verbose (OFF)", command=self._toggle_arduino_verbose_action) #
        self.verbose_button.grid(row=0, column=0, padx=5, pady=5) #
        
        self.wrist_yaw_mode_button = ttk.Checkbutton(
            sys_cmd_frame,
            text="Jaws Mode",  # Initial text, will be updated by _update_jaw_mode_ui
            variable=self.wrist_yaw_mode_var, 
            command=self._update_jaw_mode_ui,
            style="Toolbutton"
        )
        self.wrist_yaw_mode_button.grid(row=0, column=1, padx=5, pady=5) 
        
        sys_cmd_frame.columnconfigure(1, weight=1) # Adjusted column weight

        # Row 3: Inline Positional Control Frame
        self._create_positional_control_frame_widgets(self.main_content_frame) #

    def _create_positional_control_frame_widgets(self, parent_frame): #
        positional_super_frame = ttk.LabelFrame(parent_frame, text="Positional Control (Dedicated Input Fields)", padding="10") #
        positional_super_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10) #
        # parent_frame.grid_rowconfigure(3, weight=0)

        # "Move by Degrees" frame
        input_frame = ttk.LabelFrame(positional_super_frame, text="Move by Degrees", padding="10") #
        input_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ns") # Gridded to be on the left

        row_idx = 0
        ttk.Label(input_frame, text="Elbow Pitch (°):").grid(row=row_idx, column=0, sticky="w", pady=2) #
        self.ep_degrees_entry = ttk.Entry(input_frame, textvariable=self.ep_degrees_input_var, width=10) #
        self.ep_degrees_entry.grid(row=row_idx, column=1, pady=2) #
        row_idx += 1

        ttk.Label(input_frame, text="Elbow Yaw (°):").grid(row=row_idx, column=0, sticky="w", pady=2) #
        self.ey_degrees_entry = ttk.Entry(input_frame, textvariable=self.ey_degrees_input_var, width=10) #
        self.ey_degrees_entry.grid(row=row_idx, column=1, pady=2) #
        row_idx += 1

        ttk.Label(input_frame, text="Wrist Pitch (°):").grid(row=row_idx, column=0, sticky="w", pady=2) #
        self.wp_degrees_entry = ttk.Entry(input_frame, textvariable=self.wp_degrees_input_var, width=10) #
        self.wp_degrees_entry.grid(row=row_idx, column=1, pady=2) #
        row_idx += 1
        
        # Store target rows for dynamic placement by _update_jaw_mode_ui
        self.lj_target_row = row_idx
        self.rj_target_row = row_idx + 1 # Assuming RJ is one row below LJ if both are visible

        # Wrist Yaw input (hidden by default, managed by _update_jaw_mode_ui)
        self.wrist_yaw_label = ttk.Label(input_frame, text="Wrist Yaw (°):")
        self.wrist_yaw_entry = ttk.Entry(input_frame, textvariable=self.wrist_yaw_degrees_var, width=10)
        # Not gridding wrist_yaw_label/entry here; _update_jaw_mode_ui will handle it.

        # Left Jaw input (gridded here, then managed by _update_jaw_mode_ui)
        self.lj_label = ttk.Label(input_frame, text="Left Jaw (°):") #
        self.lj_degrees_entry = ttk.Entry(input_frame, textvariable=self.lj_degrees_input_var, width=10) #
        self.lj_label.grid(row=self.lj_target_row, column=0, sticky="w", pady=2) #
        self.lj_degrees_entry.grid(row=self.lj_target_row, column=1, pady=2) #
        row_idx +=1 # Increment row_idx after LJ

        # Right Jaw input (gridded here, then managed by _update_jaw_mode_ui)
        self.rj_label = ttk.Label(input_frame, text="Right Jaw (°):") #
        self.rj_degrees_entry = ttk.Entry(input_frame, textvariable=self.rj_degrees_input_var, width=10) #
        self.rj_label.grid(row=self.rj_target_row, column=0, sticky="w", pady=2) #
        self.rj_degrees_entry.grid(row=self.rj_target_row, column=1, pady=2) #
        row_idx +=1 # Increment row_idx after RJ (or use self.rj_target_row + 1 for next element)


        ttk.Button(input_frame, text="Move Joints by Degrees", command=self._dedicated_move_by_degrees_action, width=25).grid(row=row_idx, column=0, columnspan=2, pady=10) #

        # "Cumulative Joint Position" frame
        cumulative_frame = ttk.LabelFrame(positional_super_frame, text="Cumulative Joint Position (Degrees from Ref)", padding="10") #
        cumulative_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ns") # Gridded to be on the right
        
        row_idx_cum = 0
        ttk.Label(cumulative_frame, text="Elbow Pitch:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ep_degrees_var, width=8, anchor="e").grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Elbow Yaw:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ey_degrees_var, width=8, anchor="e").grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Wrist Pitch:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_wp_degrees_var, width=8, anchor="e").grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Left Jaw:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_lj_degrees_var, width=8, anchor="e").grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Right Jaw:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_rj_degrees_var, width=8, anchor="e").grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Button(cumulative_frame, text="Reset Cumulative Display", command=self._reset_cumulative_degrees_display_action, width=25).grid(row=row_idx_cum, column=0, columnspan=3, pady=10) #
        
        positional_super_frame.columnconfigure(0, weight=1) #
        positional_super_frame.columnconfigure(1, weight=1) #

        self._update_jaw_mode_ui() # Call to set initial UI state for Jaws/Wrist Yaw

    def _create_output_area(self): #
        output_frame = ttk.LabelFrame(self.main_content_frame, text="Output Log", padding="5") #
        output_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5) #
        self.main_content_frame.grid_rowconfigure(4, weight=1) #
        self.main_content_frame.grid_columnconfigure(0, weight=1) # This makes the column containing the output_frame expand
        
        self.output_text = tk.Text(output_frame, height=10, width=80) #
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S)) #
        
        output_frame.grid_rowconfigure(0, weight=1) # Make the Text widget expand within output_frame
        output_frame.grid_columnconfigure(0, weight=1) # Make the Text widget expand within output_frame
        
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview) #
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S)) #
        self.output_text['yscrollcommand'] = scrollbar.set #

    def log_message(self, message): #
        if hasattr(self, 'output_text') and self.output_text and self.output_text.winfo_exists(): #
            self.output_text.insert(tk.END, str(message) + "\n") #
            self.output_text.see(tk.END) #
        else:
            print(f"LOG (output_text not ready): {message}") #

    def _update_connection_status_display(self, message, color, connected): #
        self.status_label.config(text=f"Status: {message}", foreground=color) #
        self.connect_button.config(text="Disconnect" if connected else "Connect") #
        self.log_message(message) #

    def _handle_serial_data(self, response_data, data_type="received"): #
        self.log_message(f"Arduino: {response_data}") #
        if response_data.startswith("VERBOSE_STATE:"): #
            state = response_data.split(":")[1].strip() #
            self.is_verbose_arduino_side = (state == "1") #
            self.verbose_button.config(text=f"Toggle Arduino Verbose ({'ON' if self.is_verbose_arduino_side else 'OFF'})") # Updated text
            self.log_message(f"Arduino Verbose mode {'ON' if self.is_verbose_arduino_side else 'OFF'}") #
        
    def _handle_serial_error(self, error_message): #
        messagebox.showerror("Serial Error", error_message, parent=self.root) #
        self.log_message(f"ERROR: {error_message}") #

    def _toggle_connection_action(self): #
        if not self.serial_handler.is_connected: #
            port = self.port_entry.get() #
            self.serial_handler.connect(port) #
        else:
            self.serial_handler.disconnect() #

    def _get_slider_value(self): #
        """Gets the numerical value from the slider based on its current index."""
        try:
            selected_index = int(self.current_step_size_var.get()) #
            if 0 <= selected_index < len(config.STEP_SIZES): #
                return config.STEP_SIZES[selected_index] #
        except tk.TclError: pass 
        return config.STEP_SIZES[config.DEFAULT_STEP_SIZE_INDEX] #

    def _update_d_pad_mode_and_slider_label(self, event=None): #
        """Updates the D-Pad mode checkbutton text and the slider's increment display label."""
        if not hasattr(self, 'd_pad_increment_display_label') or not self.d_pad_increment_display_label: #
            return
        
        current_val = self._get_slider_value() #
        is_degree_mode = self.d_pad_degree_mode_var.get() #

        if is_degree_mode: #
            unit_label = "degrees" #
            self.d_pad_mode_button.config(text="D-Pad Mode: Degrees") #
        else:
            unit_label = "steps" #
            self.d_pad_mode_button.config(text="D-Pad Mode: Steps") #
        
        self.d_pad_increment_display_label.config(text=f"{current_val} {unit_label}") #

    def _update_jaw_mode_ui(self, event=None):
        """Updates the UI elements for Jaw/Wrist Yaw mode for dedicated degree inputs."""
        is_wrist_yaw_mode = self.wrist_yaw_mode_var.get()

        if hasattr(self, 'wrist_yaw_mode_button') and self.wrist_yaw_mode_button.winfo_exists():
            self.wrist_yaw_mode_button.config(text="Wrist Yaw Mode" if is_wrist_yaw_mode else "Jaws Mode")

        # Determine target row for Wrist Yaw (e.g., same as Left Jaw's original position)
        wrist_yaw_target_row = self.lj_target_row if hasattr(self, 'lj_target_row') else 3 

        # Show/Hide Left Jaw input fields
        if hasattr(self, 'lj_label') and self.lj_label.winfo_exists():
            if not is_wrist_yaw_mode:
                self.lj_label.grid(row=self.lj_target_row, column=0, sticky="w", pady=2)
                self.lj_degrees_entry.grid(row=self.lj_target_row, column=1, pady=2)
            else:
                self.lj_label.grid_remove()
                self.lj_degrees_entry.grid_remove()

        # Show/Hide Right Jaw input fields
        if hasattr(self, 'rj_label') and self.rj_label.winfo_exists():
            if not is_wrist_yaw_mode:
                self.rj_label.grid(row=self.rj_target_row, column=0, sticky="w", pady=2)
                self.rj_degrees_entry.grid(row=self.rj_target_row, column=1, pady=2)
            else:
                self.rj_label.grid_remove()
                self.rj_degrees_entry.grid_remove()

        # Show/Hide Wrist Yaw input fields
        if hasattr(self, 'wrist_yaw_label') and self.wrist_yaw_label.winfo_exists():
            if is_wrist_yaw_mode:
                self.wrist_yaw_label.grid(row=wrist_yaw_target_row, column=0, sticky="w", pady=2)
                self.wrist_yaw_entry.grid(row=wrist_yaw_target_row, column=1, pady=2)
            else:
                self.wrist_yaw_label.grid_remove()
                self.wrist_yaw_entry.grid_remove()

        if hasattr(self, 'lj_degrees_input_var'): self.lj_degrees_input_var.set("0.0")
        if hasattr(self, 'rj_degrees_input_var'): self.rj_degrees_input_var.set("0.0")
        if hasattr(self, 'wrist_yaw_degrees_var'): self.wrist_yaw_degrees_var.set("0.0")

    def _d_pad_elbow_action(self, pitch_val, yaw_val): #
        """Handles Elbow D-Pad actions based on the current D-Pad control mode."""
        if pitch_val == 0 and yaw_val == 0: return #

        if not self.d_pad_degree_mode_var.get(): # Direct Step Mode
            motor_steps = [0] * len(MotorIndex) #
            motor_steps[MotorIndex.EPD] = int(pitch_val)  # Based on image: UP arrow leads to negative pitch_val
            motor_steps[MotorIndex.EPU] = -int(pitch_val) #
            motor_steps[MotorIndex.EYR] = -int(yaw_val)    # Based on image: LEFT arrow leads to positive yaw_val
            motor_steps[MotorIndex.EYL] = int(yaw_val) #
            steps_str = ",".join(map(str, motor_steps)) #
            cmd = f"MOVE_ALL_MOTORS:{steps_str}" #
            self.serial_handler.send_command(cmd) #
            self.log_message(f"Sent D-Pad Step Elbow: Pitch {pitch_val}, Yaw {yaw_val} (Steps: {cmd})") #
        else: # Degree-Based Mode
            joint_degree_deltas = {"EP": float(pitch_val), "EY": float(yaw_val)} #
            self.log_message(f"D-Pad Degree Elbow: Pitch {pitch_val}°, Yaw {yaw_val}°") #
            self._execute_degree_based_move(joint_degree_deltas) #
            #self.cumulative_ep_degrees_var.set(self.cumulative_ep_degrees_var.get() + pitch_val)
            #self.cumulative_ey_degrees_var.set(self.cumulative_ey_degrees_var.get() + yaw_val)


    def _d_pad_wrist_action(self, value): #
        """Handles Wrist D-Pad actions based on the current D-Pad control mode."""
        if value == 0: return #

        if not self.d_pad_degree_mode_var.get(): # Direct Step Mode
            motor_steps = [0] * len(MotorIndex) #
            motor_steps[MotorIndex.WPD] = int(value)  # Based on image: UP arrow leads to positive value
            motor_steps[MotorIndex.WPU] = -int(value) #
            steps_str = ",".join(map(str, motor_steps)) #
            cmd = f"MOVE_ALL_MOTORS:{steps_str}" #
            self.serial_handler.send_command(cmd) #
            self.log_message(f"Sent D-Pad Step Wrist: {value} (Steps: {cmd})") #
        else: # Degree-Based Mode
            joint_degree_deltas = {"WP": float(value)} #
            self.log_message(f"D-Pad Degree Wrist: {value}°") #
            self._execute_degree_based_move(joint_degree_deltas) #

    def _d_pad_left_jaw_action(self, value): #
        """Handles Left Jaw D-Pad actions based on the current D-Pad control mode."""
        if value == 0: return #
        if not self.d_pad_degree_mode_var.get(): # Direct Step Mode
            motor_steps = [0] * len(MotorIndex) #
            motor_steps[MotorIndex.LJR] = int(value) #
            motor_steps[MotorIndex.LJL] = -int(value) #
            steps_str = ",".join(map(str, motor_steps)) #
            cmd = f"MOVE_ALL_MOTORS:{steps_str}" #
            self.serial_handler.send_command(cmd) #
            self.log_message(f"Sent D-Pad Step Left Jaw: {value} (Steps: {cmd})") #
        else: # Degree-Based Mode
            joint_degree_deltas = {"LJ": float(value)} #
            self.log_message(f"D-Pad Degree Left Jaw: {value}°") #
            self._execute_degree_based_move(joint_degree_deltas) #
            #self.cumulative_lj_degrees_var.set(self.cumulative_lj_degrees_var.get() + value)


    def _d_pad_right_jaw_action(self, value): #
        """Handles Right Jaw D-Pad actions based on the current D-Pad control mode."""
        if value == 0: return #
        if not self.d_pad_degree_mode_var.get(): # Direct Step Mode
            motor_steps = [0] * len(MotorIndex) #
            motor_steps[MotorIndex.RJR] = int(value) # Based on image: LEFT arrow leads to positive value
            motor_steps[MotorIndex.RJL] = -int(value) #
            steps_str = ",".join(map(str, motor_steps)) #
            cmd = f"MOVE_ALL_MOTORS:{steps_str}" #
            self.serial_handler.send_command(cmd) #
            self.log_message(f"Sent D-Pad Step Right Jaw: {value} (Steps: {cmd})") #
        else: # Degree-Based Mode
            joint_degree_deltas = {"RJ": float(value)} #
            self.log_message(f"D-Pad Degree Right Jaw: {value}°") #
            self._execute_degree_based_move(joint_degree_deltas) #
            #self.cumulative_rj_degrees_var.set(self.cumulative_rj_degrees_var.get() + value)


    def _toggle_arduino_verbose_action(self): # Renamed for clarity
        self.serial_handler.send_command("TOGGLE_VERBOSE") #
        self.log_message("Sent: TOGGLE_VERBOSE to Arduino") #

    def _dedicated_move_by_degrees_action(self): #
        """Handles the 'Move Joints by Degrees' button press from dedicated input fields."""
        try:
            if self.wrist_yaw_mode_var.get(): # Check if in Wrist Yaw mode
                jaw_val_for_yaw = float(self.wrist_yaw_degrees_var.get())
                joint_degree_deltas = {
                    "EP": float(self.ep_degrees_input_var.get()),
                    "EY": float(self.ey_degrees_input_var.get()),
                    "WP": float(self.wp_degrees_input_var.get()),
                    "LJ": jaw_val_for_yaw, 
                    "RJ": jaw_val_for_yaw, 
                }
                self.wrist_yaw_degrees_var.set("0.0")
            else: # Jaws Mode
                joint_degree_deltas = {
                    "EP": float(self.ep_degrees_input_var.get()),
                    "EY": float(self.ey_degrees_input_var.get()),
                    "WP": float(self.wp_degrees_input_var.get()),
                    "LJ": float(self.lj_degrees_input_var.get()), #
                    "RJ": float(self.rj_degrees_input_var.get()), #
                }
                self.lj_degrees_input_var.set("0.0") #
                self.rj_degrees_input_var.set("0.0") #

            # Reset common input fields
            self.ep_degrees_input_var.set("0.0") #
            self.ey_degrees_input_var.set("0.0") #
            self.wp_degrees_input_var.set("0.0") #
            # self.roll_degrees_input_var.set("0.0") # Removed Roll reset

        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.", parent=self.root) #
            return
        
        self.log_message(f"Dedicated Degree Input: Deltas {joint_degree_deltas}") #
        self._execute_degree_based_move(joint_degree_deltas) #

    def _execute_degree_based_move(self, joint_degree_deltas_input): #
        """
        Core logic to calculate and send motor commands based on degree deltas.
        joint_degree_deltas_input: A dictionary like {"EP": 10.0, "EY": -5.0, ...}
                                        Only keys for joints to be moved need to be present.
        """
        all_joint_keys = ["EP", "EY", "WP", "LJ", "RJ"] #
        full_joint_degree_deltas = {key: 0.0 for key in all_joint_keys} #
        full_joint_degree_deltas.update(joint_degree_deltas_input) #

        current_abs_positions = { #
            "EP": self.cumulative_ep_degrees_var.get(), #
            "EY": self.cumulative_ey_degrees_var.get(), #
            "WP": self.cumulative_wp_degrees_var.get(), #
            "LJ": self.cumulative_lj_degrees_var.get(), #
            "RJ": self.cumulative_rj_degrees_var.get(), #
        }

        joint_processors = { #
            "EP": (current_abs_positions["EP"], full_joint_degree_deltas["EP"], q1_pl.get_steps, self.latest_dir["EP"]), #
            "EY": (current_abs_positions["EY"], full_joint_degree_deltas["EY"], q2_pl.get_steps, self.latest_dir["EY"]), #
            "WP": (current_abs_positions["WP"], full_joint_degree_deltas["WP"], q3_pl.get_steps, self.latest_dir["WP"]), #
            "LJ": (current_abs_positions["LJ"], full_joint_degree_deltas["LJ"], q4_pl.get_steps_L, self.latest_dir["LJ"]), #
            "RJ": (current_abs_positions["RJ"], full_joint_degree_deltas["RJ"], q4_pl.get_steps_R, self.latest_dir["RJ"]), #
        }

        total_motor_steps = [0] * len(MotorIndex) #

        for joint_name, (curr_theta, delta_theta, get_steps_function, latest_dir) in joint_processors.items(): #
            if delta_theta != 0: # Process only if there's a delta for this joint #
                try:
                    self.log_message(f"Calculating for {joint_name}: current_abs={curr_theta:.2f}°, delta={delta_theta:.2f}°") #
                    joint_specific_motor_steps, self.latest_dir[joint_name] = get_steps_function(curr_theta, delta_theta, latest_dir) #
                    for motor_idx_enum in MotorIndex: # Iterate using enum members #
                        total_motor_steps[motor_idx_enum.value] += joint_specific_motor_steps[motor_idx_enum.value] #
                    self.log_message(f"  Steps from {joint_name}: {joint_specific_motor_steps}") #
                except Exception as e:
                    self.log_message(f"  Error calling {get_steps_function.__name__} for {joint_name}: {e}") #
                    import traceback #
                    self.log_message(traceback.format_exc()) #
        
        final_integer_steps = [int(round(s)) for s in total_motor_steps] #
        self.log_message(f"Final Combined Steps: {final_integer_steps}") #
                
        steps_str = ",".join(map(str, final_integer_steps)) #
        cmd = f"MOVE_ALL_MOTORS:{steps_str}" #

        if self.serial_handler.send_command(cmd): #
            self.log_message(f"Sent Degree-Based Move: {cmd}") #
            if full_joint_degree_deltas["EP"] != 0: #
                self.cumulative_ep_degrees_var.set(round(current_abs_positions["EP"] + full_joint_degree_deltas["EP"], 2)) #
            if full_joint_degree_deltas["EY"] != 0: #
                self.cumulative_ey_degrees_var.set(round(current_abs_positions["EY"] + full_joint_degree_deltas["EY"], 2)) #
            if full_joint_degree_deltas["WP"] != 0: #
                self.cumulative_wp_degrees_var.set(round(current_abs_positions["WP"] + full_joint_degree_deltas["WP"], 2)) #
            if full_joint_degree_deltas["LJ"] != 0: #
                self.cumulative_lj_degrees_var.set(round(current_abs_positions["LJ"] + full_joint_degree_deltas["LJ"], 2)) #
            if full_joint_degree_deltas["RJ"] != 0: #
                self.cumulative_rj_degrees_var.set(round(current_abs_positions["RJ"] + full_joint_degree_deltas["RJ"], 2)) #
        else:
            self.log_message("Failed to send degree-based command.") #

    def _reset_cumulative_degrees_display_action(self, from_test_mode=False, is_initial_setup=False): # Added is_initial_setup flag
        self.cumulative_ep_degrees_var.set(90.0) #
        self.cumulative_ey_degrees_var.set(90.0) #
        self.cumulative_wp_degrees_var.set(90.0) #
        self.cumulative_lj_degrees_var.set(90.0) #
        self.cumulative_rj_degrees_var.set(90.0) #
        
        if not from_test_mode and not is_initial_setup: #
            messagebox.showinfo("Reset", "Cumulative degree display has been reset to reference positions.", parent=self.root) #
        if not is_initial_setup: # Don't log during init before log widget is ready
            self.log_message("Cumulative degree display reset to reference positions.") #

    def _reset_cumulative_degrees_display_from_test_mode(self): #
        self._reset_cumulative_degrees_display_action(from_test_mode=True) #
        self.log_message("Cumulative degrees display reset due to 'Set Home' in Test Motors mode.") #

    def cleanup_on_exit(self): #
        self.log_message("Application exiting. Cleaning up...")
        
        self.serial_handler.cleanup() #
        self.log_message("Serial handler cleaned up.") #