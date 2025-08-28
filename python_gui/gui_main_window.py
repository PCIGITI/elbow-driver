import tkinter as tk
from tkinter import ttk, messagebox
import math
from datetime import datetime
import time 
import json
import os
import traceback

import config # For constants, MotorIndex
from config import MotorIndex #
import q1_pl, q2_pl, q3_pl, q4_pl # Joint processors for step calculations

# ROS-related imports with a fallback if ROS is not installed
import queue
import threading
try:
    import rospy
    # NEW: Import JointState message type
    from sensor_msgs.msg import JointState
    IS_ROS_AVAILABLE = True
except ImportError:
    IS_ROS_AVAILABLE = False

# --- Helper Class for ROS Communication ---
class ROSSubscriberThread(threading.Thread):
    def __init__(self, topic_name, data_queue, log_callback, min_interval_sec=0.5):
        super().__init__(daemon=True)
        self._topic_name = topic_name
        self._data_queue = data_queue
        self._log_callback = log_callback
        self._subscriber = None
        self._stop_event = threading.Event()
        self._last_processed_time = 0
        self._min_interval_sec = min_interval_sec  # Minimum time between processing messages

    def _ros_callback(self, msg):
        now = time.time()
        if now - self._last_processed_time < self._min_interval_sec:
            return  # Skip this message
        self._last_processed_time = now

        try:
            # ...existing message processing code...
            joint_map = dict(zip(msg.name, msg.position))
            required_joints = ["elbow_pitch", "elbow_yaw", "wrist_pitch", "jaw_1"]
            if not all(joint in joint_map for joint in required_joints):
                missing = [j for j in required_joints if j not in joint_map]
                self._log_callback(f"ROS WARNING: Missing joints in message: {missing}", level="warning")
                return
            def convert_rad_to_deg(rad_val):
                return math.degrees(rad_val) + 90.0
            target_positions_deg = {
                "Q1": convert_rad_to_deg(joint_map["elbow_pitch"]),
                "Q2": convert_rad_to_deg(joint_map["elbow_yaw"]),
                "Q3": convert_rad_to_deg(joint_map["wrist_pitch"]),
                "Q4L": convert_rad_to_deg(joint_map["jaw_1"]),
                "Q4R": -math.degrees(joint_map["jaw_1"]) + 90.0
            }
            self._data_queue.put(target_positions_deg)
        except Exception as e:
            self._log_callback(f"ROS Callback Error: {e}", level="error")

    def run(self):
        """The main execution method of the thread."""
        try:
            self._log_callback(f"ROS Thread: Subscribing to topic '{self._topic_name}'.")
            self._subscriber = rospy.Subscriber(self._topic_name, JointState, self._ros_callback)
            
            # Loop until the stop event is set
            while not self._stop_event.is_set() and not rospy.is_shutdown():
                # wait() with a timeout acts like a sleep but is interruptible
                self._stop_event.wait(0.1)

        except rospy.ROSInterruptException:
            self._log_callback("ROS Thread: Shutdown signal received.")
        except Exception as e:
            self._log_callback(f"ROS Thread: An error occurred during run: {e}", level="error")
        finally:
            # Unregister here to ensure it happens as the thread exits.
            if self._subscriber:
                self._subscriber.unregister()
                self._log_callback("ROS Thread: Unsubscribed from topic.")
            self._log_callback("ROS Thread: Exiting.")

    def stop(self):
        """Signals the thread's run loop to terminate."""
        self._log_callback("ROS Thread: Stop signal received.")
        self._stop_event.set()


class ElbowSimulatorGUI:
    def __init__(self, root, serial_handler): #
        self.root = root #
        self.serial_handler = serial_handler #

        self.root.title(config.APP_TITLE) #
        self.root.geometry(config.MAIN_WINDOW_GEOMETRY) #
        
        # --- NEW: Setup Cyberpunk Theme ---
        self._setup_cyberpunk_style()

        self.serial_handler.set_callbacks( #
            data_callback=self._handle_serial_data, #
            status_callback=self._update_connection_status_display, #
            error_callback=self._handle_serial_error #
        )
        self.is_verbose_arduino_side = False #

        # --- Tkinter Variables ---
        self.control_mode_is_degrees_var = tk.BooleanVar(value=True) # True=Degrees, False=Steps
        self.individual_motor_mode_var = tk.BooleanVar(value=False)
        self.step_degree_input_var = tk.StringVar(value="1.0")
        
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
        
        # --- ROS Variables ---
        self.ros_mode_var = tk.BooleanVar(value=False)
        self.ros_topic_var = tk.StringVar(value="/joint_states") # Default topic
        self.ros_status_var = tk.StringVar(value="Status: Inactive")
        self.ros_update_freq_ms = tk.IntVar(value=100) # NEW: Update frequency in ms
        self.ros_queue = queue.Queue()
        self.ros_thread = None
        self.ros_node_initialized = False # Flag to ensure rospy.init_node is called only once

        # --- Joint Limit Variables ---
        self.theta1_min_input_var = tk.StringVar(value="0")
        self.theta1_max_input_var = tk.StringVar(value="4095")
        self.theta2_min_input_var = tk.StringVar(value="0")
        self.theta2_max_input_var = tk.StringVar(value="4095")
        self.theta1_min_display_var = tk.StringVar(value="N/A")
        self.theta1_max_display_var = tk.StringVar(value="N/A")
        self.theta2_min_display_var = tk.StringVar(value="N/A")
        self.theta2_max_display_var = tk.StringVar(value="N/A")

        # --- Sign Convention Variables ---
        self.Q1_invert = 1
        self.Q2_invert = 1
        self.Q3_invert = 1
        self.Q4L_invert = 1
        self.Q4R_invert = 1
        self.q1_dir_var = tk.StringVar(value="South")
        self.q2_dir_var = tk.StringVar(value="West")
        self.q3_dir_var = tk.StringVar(value="South")
        self.q4l_dir_var = tk.StringVar(value="West")
        self.q4r_dir_var = tk.StringVar(value="East")
        self.invert_buttons = {} # NEW: Dictionary to hold references to invert buttons

        #holds the latest direction values for each motor pair, 0 for true neutral, 1 for ccw, -1 for cw
        self.latest_dir = {
            "EP": 0,
            "EY": 0,
            "WP": 0,
            "LJ": 0,
            "RJ": 0,
        }

        self._load_settings()
        self._setup_main_layout() #
        self._create_widgets() #
        self._create_output_area() # Create the serial output log box
        self._update_control_mode_ui() # Initialize new control UI
        self._check_ros_queue() # Start the loop to check for ROS messages

        self.root.protocol("WM_DELETE_WINDOW", self.cleanup_on_exit)

        self.log_message(f"// {config.APP_TITLE} INITIALIZED //", level="info")

    def _load_settings(self):
        """Loads settings from a JSON file at startup."""
        settings_path = os.path.join(os.path.dirname(__file__), "gui_settings.json")
        
        try:
            with open(settings_path, "r") as f:
                settings = json.load(f)
                
                # Load limit values, providing defaults if a key is missing
                self.theta1_min_input_var.set(settings.get("theta1_min", "0"))
                self.theta1_max_input_var.set(settings.get("theta1_max", "4095"))
                self.theta2_min_input_var.set(settings.get("theta2_min", "0"))
                self.theta2_max_input_var.set(settings.get("theta2_max", "4095"))

                # Update the "Current" display labels as well
                self.theta1_min_display_var.set(self.theta1_min_input_var.get())
                self.theta1_max_display_var.set(self.theta1_max_input_var.get())
                self.theta2_min_display_var.set(self.theta2_min_input_var.get())
                self.theta2_max_display_var.set(self.theta2_max_input_var.get())

                # Load invert values
                self.Q1_invert = 1 if settings.get("q1_invert", "0") == "0" else -1
                self.Q2_invert = 1 if settings.get("q2_invert", "0") == "0" else -1
                self.Q3_invert = 1 if settings.get("q3_invert", "0") == "0" else -1
                self.Q4L_invert = 1 if settings.get("q4L_invert", "0") == "0" else -1
                self.Q4R_invert = 1 if settings.get("q4R_invert", "0") == "0" else -1

                # Update direction variables based on loaded invert values
                self.q1_dir_var.set("North" if self.Q1_invert == -1 else "South")
                self.q2_dir_var.set("East" if self.Q2_invert == -1 else "West")
                self.q3_dir_var.set("North" if self.Q3_invert == -1 else "South")
                self.q4l_dir_var.set("East" if self.Q4L_invert == -1 else "West")
                self.q4r_dir_var.set("West" if self.Q4R_invert == -1 else "East")
                
                self.log_message("Loaded settings from gui_settings.json.", level="info")
                self.root.update_idletasks()

        except FileNotFoundError:
            self.log_message("No settings file found. Using default values.", level="warning")
        except Exception as e:
            self.log_message(f"Error loading settings: {e}", level="error")

    def _save_settings(self):
        """Saves current settings to a JSON file."""
        settings = {
            "theta1_min": self.theta1_min_input_var.get(),
            "theta1_max": self.theta1_max_input_var.get(),
            "theta2_min": self.theta2_min_input_var.get(),
            "theta2_max": self.theta2_max_input_var.get(),
            "q1_invert": "1" if self.Q1_invert == -1 else "0",
            "q2_invert": "1" if self.Q2_invert == -1 else "0",
            "q3_invert": "1" if self.Q3_invert == -1 else "0",
            "q4L_invert": "1" if self.Q4L_invert == -1 else "0",
            "q4R_invert": "1" if self.Q4R_invert == -1 else "0"
        }
        try:
            settings_path = os.path.join(os.path.dirname(__file__), "gui_settings.json")
            with open(settings_path, "w") as f:
                json.dump(settings, f, indent=4)
            self.log_message("Settings saved to gui_settings.json.", level="info")
        except Exception as e:
            self.log_message(f"Error saving settings: {e}", level="error")

    def _setup_cyberpunk_style(self):
        """Configures the ttk styles for a cyberpunk hacker theme."""
        # --- Color Palette & Fonts ---
        self.BG_COLOR = "#0D0208"       # Near-black purple
        self.FG_COLOR = "#00FF9B"       # Bright neon mint/cyan
        self.TEXT_COLOR = "#EAEAEA"     # Off-white
        self.ALT_BG_COLOR = "#1a1a1a"   # Dark gray for entry fields/buttons
        self.BORDER_COLOR = "#FF00FF"   # Magenta for highlights
        
        self.INFO_COLOR = "#00FF9B"     # Neon Mint
        self.SENT_COLOR = "#00BFFF"     # Deep Sky Blue
        self.RECEIVED_COLOR = "#F0E68C" # Khaki/Yellowish
        self.ERROR_COLOR = "#FF4136"    # Red
        self.WARNING_COLOR = "#FF851B"  # Orange
        self.TIMESTAMP_COLOR = "#888888" # Gray

        self.FONT_HACKER = ("Consolas", 11)
        self.FONT_HACKER_BOLD = ("Consolas", 11, "bold")

        # --- Style Configuration ---
        style = ttk.Style(self.root)
        self.root.configure(background=self.BG_COLOR)
        style.theme_use('clam')

        # --- Base element styles ---
        style.configure('.',
                        background=self.BG_COLOR,
                        foreground=self.TEXT_COLOR,
                        font=self.FONT_HACKER,
                        fieldbackground=self.ALT_BG_COLOR,
                        borderwidth=0,
                        lightcolor=self.BORDER_COLOR,
                        darkcolor=self.BG_COLOR)

        # --- Frame styles ---
        style.configure('TFrame', background=self.BG_COLOR)
        style.configure('TLabelframe',
                        background=self.BG_COLOR,
                        borderwidth=1,
                        relief="solid")
        style.configure('TLabelframe.Label',
                        background=self.BG_COLOR,
                        foreground=self.FG_COLOR,
                        font=self.FONT_HACKER_BOLD)

        # --- Button styles ---
        style.configure('TButton',
                        background=self.ALT_BG_COLOR,
                        foreground=self.FG_COLOR,
                        font=self.FONT_HACKER_BOLD,
                        borderwidth=1,
                        relief='flat',
                        padding=6)
        style.map('TButton',
                  background=[('active', self.FG_COLOR), ('pressed', self.FG_COLOR)],
                  foreground=[('active', self.BG_COLOR), ('pressed', self.BG_COLOR)])
        
        # --- NEW: Style for the "Reset" button state ---
        style.configure('Reset.TButton',
                        background=self.BORDER_COLOR,
                        foreground=self.TEXT_COLOR,
                        font=self.FONT_HACKER_BOLD,
                        borderwidth=1,
                        relief='flat',
                        padding=6)
        style.map('Reset.TButton',
                  background=[('active', self.FG_COLOR), ('pressed', self.FG_COLOR)],
                  foreground=[('active', self.BG_COLOR), ('pressed', self.BG_COLOR)])

        # --- Checkbutton (Toolbutton style) ---
        style.configure('Toolbutton',
                        background=self.ALT_BG_COLOR,
                        foreground=self.FG_COLOR,
                        font=self.FONT_HACKER,
                        padding=6,
                        relief='flat')
        style.map('Toolbutton',
                  background=[('selected', self.BORDER_COLOR), ('active', self.FG_COLOR)],
                  foreground=[('selected', self.TEXT_COLOR), ('active', self.BG_COLOR)])

        # --- Entry widget style ---
        style.configure('TEntry',
                        fieldbackground=self.ALT_BG_COLOR,
                        foreground=self.FG_COLOR,
                        insertcolor=self.FG_COLOR, # Cursor color
                        borderwidth=1,
                        relief='flat')
                        
        # --- Scrollbar style ---
        style.configure('Vertical.TScrollbar',
                background=self.ALT_BG_COLOR,
                troughcolor=self.BG_COLOR,
                borderwidth=0,
                arrowcolor=self.FG_COLOR)
        style.map('Vertical.TScrollbar',
                background=[('active', self.FG_COLOR)])

    def _setup_main_layout(self): #
        self.canvas = tk.Canvas(self.root, borderwidth=0, background=self.BG_COLOR, highlightthickness=0)
        self.v_scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview, style='Vertical.TScrollbar')
        self.canvas.configure(yscrollcommand=self.v_scrollbar.set) #
        self.v_scrollbar.pack(side="right", fill="y") #
        self.canvas.pack(side="left", fill="both", expand=True) #

        self.container_frame = ttk.Frame(self.canvas, style='TFrame')
        self.main_frame_id = self.canvas.create_window((0, 0), window=self.container_frame, anchor="nw") #

        self.main_content_frame = ttk.Frame(self.container_frame, padding="10", style='TFrame')
        self.main_content_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S)) #
        self.container_frame.columnconfigure(0, weight=1) #

        self.container_frame.bind("<Configure>", self._on_frame_configure) #
        self.canvas.bind("<Configure>", self._on_canvas_configure) #

    def _on_frame_configure(self, event): #
        self.canvas.configure(scrollregion=self.canvas.bbox("all")) #

    def _on_canvas_configure(self, event): #
        self.canvas.itemconfig(self.main_frame_id, width=event.width) #

    def _create_widgets(self): #
        # Top frame to hold Connection and System Ops side-by-side
        top_frame = ttk.Frame(self.main_content_frame)
        top_frame.grid(row=0, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        top_frame.columnconfigure(0, weight=1)
        top_frame.columnconfigure(1, weight=1)

        # Column 0: Connection
        conn_frame = ttk.LabelFrame(top_frame, text="<CONNECTION_LINK>", padding="10")
        conn_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        ttk.Label(conn_frame, text="PORT:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(conn_frame, width=12, font=self.FONT_HACKER)
        self.port_entry.insert(0, config.DEFAULT_SERIAL_PORT)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="[CONNECT]", command=self._toggle_connection_action)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        self.status_label = ttk.Label(conn_frame, text="STATUS: STANDBY")
        self.status_label.grid(row=0, column=3, padx=10, pady=5, sticky=tk.W)
        conn_frame.columnconfigure(3, weight=1)

        # Column 1: System Commands
        sys_cmd_frame = ttk.LabelFrame(top_frame, text="<SYSTEM_OPS>", padding="10")
        sys_cmd_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        self.verbose_button = ttk.Button(sys_cmd_frame, text="VERBOSITY: OFF", command=self._toggle_arduino_verbose_action)
        self.verbose_button.grid(row=0, column=0, padx=5, pady=5)
        self.wrist_yaw_mode_button = ttk.Checkbutton(
            sys_cmd_frame,
            text="MODE: JAWS",
            variable=self.wrist_yaw_mode_var,
            command=self._update_jaw_mode_ui,
            style="Toolbutton"
        )
        self.wrist_yaw_mode_button.grid(row=0, column=1, padx=15, pady=5)
        sys_cmd_frame.columnconfigure(1, weight=1)

        # Row 1: Holds both Manual Control and Sign Conventions
        controls_container_frame = ttk.Frame(self.main_content_frame)
        controls_container_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        controls_container_frame.columnconfigure(0, weight=1) # Manual controls
        controls_container_frame.columnconfigure(1, weight=1) # Sign conventions

        # Create the two frames inside the container
        self._create_joint_control_widgets(controls_container_frame)
        self._create_sign_conventions_widgets(controls_container_frame)

        # Row 2: ROS Control Frame
        self._create_ros_control_widgets(self.main_content_frame)

        # Row 3: Elbow Joint Limits Frame
        self._create_joint_limits_widgets(self.main_content_frame)

        # Row 4: Inline Positional Control Frame
        self._create_positional_control_frame_widgets(self.main_content_frame)
    
    def _create_ros_control_widgets(self, parent_frame):
        """Creates the widgets for the ROS control feature."""
        ros_frame = ttk.LabelFrame(parent_frame, text="<ROS_INTERFACE>", padding="10")
        ros_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

        self.ros_mode_toggle = ttk.Checkbutton(
            ros_frame,
            text="ROS_MODE",
            variable=self.ros_mode_var,
            command=self._toggle_ros_mode,
            style="Toolbutton"
        )
        self.ros_mode_toggle.grid(row=0, column=0, padx=5, pady=5)

        ttk.Label(ros_frame, text="TOPIC:").grid(row=0, column=1, padx=(15, 0), pady=5)
        self.ros_topic_entry = ttk.Entry(ros_frame, textvariable=self.ros_topic_var, width=25, font=self.FONT_HACKER)
        self.ros_topic_entry.grid(row=0, column=2, padx=5, pady=5)

        self.ros_subscribe_button = ttk.Button(
            ros_frame, text="[SUBSCRIBE]", command=self._start_ros_subscriber
        )
        self.ros_subscribe_button.grid(row=0, column=3, padx=5, pady=5)

        # NEW: Update frequency entry
        ttk.Label(ros_frame, text="FREQ (ms):").grid(row=0, column=4, padx=(15, 0), pady=5)
        self.ros_freq_entry = ttk.Entry(ros_frame, textvariable=self.ros_update_freq_ms, width=8, font=self.FONT_HACKER)
        self.ros_freq_entry.grid(row=0, column=5, padx=5, pady=5)

        self.ros_status_label = ttk.Label(ros_frame, textvariable=self.ros_status_var)
        self.ros_status_label.grid(row=0, column=6, padx=10, pady=5, sticky=tk.W)
        
        ros_frame.columnconfigure(6, weight=1)

        self.ros_topic_entry.config(state=tk.DISABLED)
        self.ros_subscribe_button.config(state=tk.DISABLED)
        self.ros_freq_entry.config(state=tk.DISABLED)

    def _create_joint_limits_widgets(self, parent_frame):
        """Creates the widgets for setting and finding elbow joint limits."""
        limits_frame = ttk.LabelFrame(parent_frame, text="<ELBOW_JOINT_LIMITS>", padding="10")
        limits_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)

        # Use a sub-frame for better alignment
        fields_frame = ttk.Frame(limits_frame)
        fields_frame.grid(row=0, column=0, padx=5, pady=5)

        # --- Input Fields ---
        # Theta 1 Min
        ttk.Label(fields_frame, text="Theta 1 Min:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(fields_frame, textvariable=self.theta1_min_input_var, width=10, font=self.FONT_HACKER).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(fields_frame, text="Current:").grid(row=0, column=2, sticky="e", padx=(10, 2))
        ttk.Label(fields_frame, textvariable=self.theta1_min_display_var, foreground=self.FG_COLOR).grid(row=0, column=3, sticky="w")
        
        # Theta 1 Max
        ttk.Label(fields_frame, text="Theta 1 Max:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(fields_frame, textvariable=self.theta1_max_input_var, width=10, font=self.FONT_HACKER).grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(fields_frame, text="Current:").grid(row=1, column=2, sticky="e", padx=(10, 2))
        ttk.Label(fields_frame, textvariable=self.theta1_max_display_var, foreground=self.FG_COLOR).grid(row=1, column=3, sticky="w")

        # Theta 2 Min
        ttk.Label(fields_frame, text="Theta 2 Min:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(fields_frame, textvariable=self.theta2_min_input_var, width=10, font=self.FONT_HACKER).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(fields_frame, text="Current:").grid(row=2, column=2, sticky="e", padx=(10, 2))
        ttk.Label(fields_frame, textvariable=self.theta2_min_display_var, foreground=self.FG_COLOR).grid(row=2, column=3, sticky="w")

        # Theta 2 Max
        ttk.Label(fields_frame, text="Theta 2 Max:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        ttk.Entry(fields_frame, textvariable=self.theta2_max_input_var, width=10, font=self.FONT_HACKER).grid(row=3, column=1, padx=5, pady=2)
        ttk.Label(fields_frame, text="Current:").grid(row=3, column=2, sticky="e", padx=(10, 2))
        ttk.Label(fields_frame, textvariable=self.theta2_max_display_var, foreground=self.FG_COLOR).grid(row=3, column=3, sticky="w")
        
        # --- Buttons ---
        buttons_frame = ttk.Frame(limits_frame)
        buttons_frame.grid(row=0, column=1, sticky="ns", padx=(20, 5))
        
        send_button = ttk.Button(buttons_frame, text="[Send New Values]", command=self._send_update_limits_action)
        send_button.pack(pady=5, fill=tk.X)
        
        find_button = ttk.Button(buttons_frame, text="[Find New Values]", command=self._send_find_limits_action)
        find_button.pack(pady=5, fill=tk.X)
        
        limits_frame.columnconfigure(0, weight=1)
        limits_frame.columnconfigure(1, weight=0)

    def _create_joint_control_widgets(self, parent_frame):
        """Creates the new joint control UI as requested."""
        joint_super_frame = ttk.LabelFrame(parent_frame, text="<MANUAL_JOINT_CONTROL>", padding="10")
        joint_super_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        joint_super_frame.columnconfigure(0, weight=1)

        top_controls_frame = ttk.Frame(joint_super_frame)
        top_controls_frame.pack(pady=5, padx=5)

        self.control_mode_button = ttk.Checkbutton(
            top_controls_frame,
            text="MODE: DEGREES",
            variable=self.control_mode_is_degrees_var,
            command=self._update_control_mode_ui,
            style="Toolbutton"
        )
        self.control_mode_button.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(top_controls_frame, text="VALUE:").pack(side=tk.LEFT, padx=(0, 5))
        self.step_degree_entry = ttk.Entry(top_controls_frame, textvariable=self.step_degree_input_var, width=10, font=self.FONT_HACKER)
        self.step_degree_entry.pack(side=tk.LEFT, padx=(0, 15))

        self.individual_motor_toggle = ttk.Checkbutton(
            top_controls_frame,
            text="INDIVIDUAL_MOTOR_CTRL",
            variable=self.individual_motor_mode_var,
            style="Toolbutton",
            command=self._on_individual_motor_toggle
        )
        self.individual_motor_toggle.pack(side=tk.LEFT, padx=(0, 10))

        joints_frame = ttk.Frame(joint_super_frame, padding="5")
        joints_frame.pack(pady=(10,0))

        joint_data = [
            ("Q1 (Elbow Pitch)", "Q1"),
            ("Q2 (Elbow Yaw)", "Q2"),
            ("Q3 (Wrist Pitch)", "Q3"),
        ]

        for i, (label, joint_id) in enumerate(joint_data):
            neg_btn = ttk.Button(joints_frame, text="[-]", width=4, command=lambda j=joint_id: self._joint_button_action(j, -1))
            neg_btn.grid(row=i, column=0, padx=5, pady=2)
            ttk.Label(joints_frame, text=label, width=22, anchor="center").grid(row=i, column=1)
            ttk.Button(joints_frame, text="[+]", width=4, command=lambda j=joint_id: self._joint_button_action(j, 1)).grid(row=i, column=2, padx=5, pady=2)
            
            if joint_id == "Q1": self.q1_neg_button = neg_btn
            if joint_id == "Q2": self.q2_neg_button = neg_btn

        q4_frame = ttk.Frame(joints_frame)
        q4_frame.grid(row=len(joint_data), column=0, columnspan=3, pady=5)

        ttk.Button(q4_frame, text="[+]", width=4, command=lambda: self._joint_button_action("Q4L", 1)).grid(row=0, column=0)
        ttk.Label(q4_frame, text="Q4L", anchor="center").grid(row=0, column=1, padx=(2, 8))
        ttk.Button(q4_frame, text="[-]", width=4, command=lambda: self._joint_button_action("Q4L", -1)).grid(row=0, column=2)

        ttk.Label(q4_frame, text=" (Jaws) ", anchor="center").grid(row=0, column=3, padx=15)

        ttk.Button(q4_frame, text="[-]", width=4, command=lambda: self._joint_button_action("Q4R", -1)).grid(row=0, column=4)
        ttk.Label(q4_frame, text="Q4R", anchor="center").grid(row=0, column=5, padx=(2, 8))
        ttk.Button(q4_frame, text="[+]", width=4, command=lambda: self._joint_button_action("Q4R", 1)).grid(row=0, column=6)

    def _create_sign_conventions_widgets(self, parent_frame):
        """Creates the sign convention display and control widgets."""
        sign_frame = ttk.LabelFrame(parent_frame, text="<SIGN_CONVENTIONS>", padding="10")
        sign_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))

        # --- Table Headers ---
        ttk.Label(sign_frame, text="Joint Num", font=self.FONT_HACKER_BOLD).grid(row=0, column=0, padx=5, pady=(0, 5))
        ttk.Label(sign_frame, text="Positive Dir", font=self.FONT_HACKER_BOLD).grid(row=0, column=1, padx=5, pady=(0, 5))

        # --- Table Data ---
        joint_data = [
            ("Q1", self.q1_dir_var),
            ("Q2", self.q2_dir_var),
            ("Q3", self.q3_dir_var),
            ("Q4L", self.q4l_dir_var),
            ("Q4R", self.q4r_dir_var),
        ]

        for i, (joint_id, dir_var) in enumerate(joint_data, start=1):
            ttk.Label(sign_frame, text=joint_id, anchor="center").grid(row=i, column=0, pady=4)
            ttk.Label(sign_frame, textvariable=dir_var, width=6, anchor="center", foreground=self.FG_COLOR).grid(row=i, column=1, pady=4)
            
            # Create and store the button
            invert_button = ttk.Button(
                sign_frame,
                text="[Invert]",
                width=8,
                command=lambda j=joint_id: self._invert_direction(j)
            )
            invert_button.grid(row=i, column=2, padx=5, pady=4)
            self.invert_buttons[joint_id] = invert_button

    def _invert_direction(self, joint_id):
        """Handles the logic for inverting a joint's positive direction."""
        inversion_map = {
            "Q1": (self.q1_dir_var, ("South", "North")),
            "Q2": (self.q2_dir_var, ("West", "East")),
            "Q3": (self.q3_dir_var, ("South", "North")),
            "Q4L": (self.q4l_dir_var, ("West", "East")),
            "Q4R": (self.q4r_dir_var, ("East", "West")),
        }
        
        if joint_id in inversion_map:
            dir_var, (dir1, dir2) = inversion_map[joint_id]
            current_dir = dir_var.get()
            new_dir = dir2 if current_dir == dir1 else dir1
            dir_var.set(new_dir)
            
            # Update the corresponding invert variable
            if joint_id == "Q1": self.Q1_invert *= -1
            elif joint_id == "Q2": self.Q2_invert *= -1
            elif joint_id == "Q3": self.Q3_invert *= -1
            elif joint_id == "Q4L": self.Q4L_invert *= -1
            elif joint_id == "Q4R": self.Q4R_invert *= -1
            
            self.log_message(f"Sign convention for {joint_id} inverted to '{new_dir}'.")
            # Update the button's appearance
            self._update_invert_button_style(joint_id)
            # Save the new settings
            self._save_settings()
            
    def _update_invert_button_style(self, joint_id):
        """Updates the text and style of an invert/reset button based on its state."""
        invert_var_map = {
            "Q1": self.Q1_invert,
            "Q2": self.Q2_invert,
            "Q3": self.Q3_invert,
            "Q4L": self.Q4L_invert,
            "Q4R": self.Q4R_invert,
        }
        
        button = self.invert_buttons.get(joint_id)
        invert_state = invert_var_map.get(joint_id)

        if not button:
            return

        if invert_state == -1: # Inverted state
            button.config(text="[RESET]", style="Reset.TButton")
        else: # Default state
            button.config(text="[Invert]", style="TButton")

    def _create_positional_control_frame_widgets(self, parent_frame): #
        positional_super_frame = ttk.LabelFrame(parent_frame, text="<POSITIONAL_CONTROL>", padding="10")
        positional_super_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10) #

        input_frame = ttk.LabelFrame(positional_super_frame, text="<Move by Degrees>", padding="10")
        input_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ns") #

        row_idx = 0
        ttk.Label(input_frame, text="Elbow Pitch (°):").grid(row=row_idx, column=0, sticky="w", pady=3)
        self.ep_degrees_entry = ttk.Entry(input_frame, textvariable=self.ep_degrees_input_var, width=10, font=self.FONT_HACKER)
        self.ep_degrees_entry.grid(row=row_idx, column=1, pady=3) #
        row_idx += 1

        ttk.Label(input_frame, text="Elbow Yaw (°):").grid(row=row_idx, column=0, sticky="w", pady=3)
        self.ey_degrees_entry = ttk.Entry(input_frame, textvariable=self.ey_degrees_input_var, width=10, font=self.FONT_HACKER)
        self.ey_degrees_entry.grid(row=row_idx, column=1, pady=3) #
        row_idx += 1

        ttk.Label(input_frame, text="Wrist Pitch (°):").grid(row=row_idx, column=0, sticky="w", pady=3)
        self.wp_degrees_entry = ttk.Entry(input_frame, textvariable=self.wp_degrees_input_var, width=10, font=self.FONT_HACKER)
        self.wp_degrees_entry.grid(row=row_idx, column=1, pady=3) #
        row_idx += 1
        
        self.lj_target_row = row_idx
        self.rj_target_row = row_idx + 1 

        self.wrist_yaw_label = ttk.Label(input_frame, text="Wrist Yaw (°):")
        self.wrist_yaw_entry = ttk.Entry(input_frame, textvariable=self.wrist_yaw_degrees_var, width=10, font=self.FONT_HACKER)

        self.lj_label = ttk.Label(input_frame, text="Left Jaw (°):") #
        self.lj_degrees_entry = ttk.Entry(input_frame, textvariable=self.lj_degrees_input_var, width=10, font=self.FONT_HACKER)
        self.lj_label.grid(row=self.lj_target_row, column=0, sticky="w", pady=3) #
        self.lj_degrees_entry.grid(row=self.lj_target_row, column=1, pady=3) #
        row_idx +=1 

        self.rj_label = ttk.Label(input_frame, text="Right Jaw (°):") #
        self.rj_degrees_entry = ttk.Entry(input_frame, textvariable=self.rj_degrees_input_var, width=10, font=self.FONT_HACKER)
        self.rj_label.grid(row=self.rj_target_row, column=0, sticky="w", pady=3) #
        self.rj_degrees_entry.grid(row=self.rj_target_row, column=1, pady=3) #
        row_idx +=1 


        ttk.Button(input_frame, text="[EXECUTE_MOVE]", command=self._dedicated_move_by_degrees_action, width=25).grid(row=row_idx, column=0, columnspan=2, pady=10) #

        cumulative_frame = ttk.LabelFrame(positional_super_frame, text="<CUMULATIVE_POSITION>", padding="10")
        cumulative_frame.grid(row=0, column=1, padx=5, pady=5, sticky="ns") #
        
        row_idx_cum = 0
        ttk.Label(cumulative_frame, text="Elbow Pitch:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ep_degrees_var, width=8, anchor="e", foreground=self.FG_COLOR).grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Elbow Yaw:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_ey_degrees_var, width=8, anchor="e", foreground=self.FG_COLOR).grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Wrist Pitch:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_wp_degrees_var, width=8, anchor="e", foreground=self.FG_COLOR).grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Left Jaw:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_lj_degrees_var, width=8, anchor="e", foreground=self.FG_COLOR).grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Label(cumulative_frame, text="Right Jaw:").grid(row=row_idx_cum, column=0, sticky="w", pady=2) #
        ttk.Label(cumulative_frame, textvariable=self.cumulative_rj_degrees_var, width=8, anchor="e", foreground=self.FG_COLOR).grid(row=row_idx_cum, column=1, sticky="e", pady=2) #
        ttk.Label(cumulative_frame, text="°").grid(row=row_idx_cum, column=2, sticky="w", pady=2) #
        row_idx_cum += 1
        ttk.Button(cumulative_frame, text="[RESET_POSITION]", command=self._reset_cumulative_degrees_display_action, width=25).grid(row=row_idx_cum, column=0, columnspan=3, pady=10) #
        
        positional_super_frame.columnconfigure(0, weight=1) #
        positional_super_frame.columnconfigure(1, weight=1) #

        self._update_jaw_mode_ui() #

    def _create_output_area(self): #
        output_frame = ttk.LabelFrame(self.main_content_frame, text="<OUTPUT_LOG>", padding="5") #
        output_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5) #
        self.main_content_frame.grid_rowconfigure(5, weight=1) #
        self.main_content_frame.grid_columnconfigure(0, weight=1) #
        
        # Using a standard tk.Text widget for full color control
        self.output_text = tk.Text(output_frame, height=15, width=80,
                                   background=self.BG_COLOR,
                                   foreground=self.FG_COLOR,
                                   insertbackground=self.BORDER_COLOR, # Cursor color
                                   selectbackground=self.BORDER_COLOR,
                                   selectforeground=self.TEXT_COLOR,
                                   borderwidth=0,
                                   highlightthickness=0,
                                   font=self.FONT_HACKER)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S)) #
        
        # --- NEW: Configure color tags for the Text widget ---
        self.output_text.tag_config("info", foreground=self.INFO_COLOR)
        self.output_text.tag_config("sent", foreground=self.SENT_COLOR)
        self.output_text.tag_config("received", foreground=self.RECEIVED_COLOR)
        self.output_text.tag_config("error", foreground=self.ERROR_COLOR, font=self.FONT_HACKER_BOLD)
        self.output_text.tag_config("warning", foreground=self.WARNING_COLOR)
        self.output_text.tag_config("timestamp", foreground=self.TIMESTAMP_COLOR)
        
        output_frame.grid_rowconfigure(0, weight=1) #
        output_frame.grid_columnconfigure(0, weight=1) #
        
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview, style='Vertical.TScrollbar')
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S)) #
        self.output_text['yscrollcommand'] = scrollbar.set #

    def log_message(self, message, level="info"):
        """
        Logs a message to the output text box with timestamp, level icon, and color.
        Levels: info, sent, received, error, warning
        """
        if not (hasattr(self, 'output_text') and self.output_text and self.output_text.winfo_exists()):
            print(f"LOG ({level}): {message}")
            return

        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        level_icons = {
            "info": "##",
            "sent": ">>",
            "received": "<<",
            "error": "!!",
            "warning": "!!",
        }
        icon = level_icons.get(level, "##")

        # Insert content with specific tags for coloring
        self.output_text.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.output_text.insert(tk.END, f"{icon} {message}\n", level)
        self.output_text.see(tk.END) # Auto-scroll to the end

    def _save_logs_to_file(self):
        """Saves the content of the output log to a timestamped text file."""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"elbow_gui_log_{timestamp}.txt"
            log_content = self.output_text.get("1.0", tk.END)
            
            with open(filename, "w") as f:
                f.write(log_content)
            
            self.log_message(f"Log saved to {filename}", level="info")
        except Exception as e:
            self.log_message(f"Failed to save log file: {e}", level="error")

    def _update_connection_status_display(self, message, color, connected): #
        self.status_label.config(text=f"STATUS: {message}") #
        self.connect_button.config(text="[DISCONNECT]" if connected else "[CONNECT]") #
        self.log_message(message) #
        
    def _handle_serial_data(self, response_data, data_type="received"): #
        self.log_message(f"Arduino: {response_data}", level="received")
        if response_data.startswith("VERBOSE_STATE:"): #
            state = response_data.split(":")[1].strip() #
            self.is_verbose_arduino_side = (state == "1") #
            self.verbose_button.config(text=f"VERBOSITY: {'ON' if self.is_verbose_arduino_side else 'OFF'}")
            self.log_message(f"Arduino Verbose mode {'ON' if self.is_verbose_arduino_side else 'OFF'}") #
        
    def _handle_serial_error(self, error_message):
        """Handles serial errors, with special handling for disconnections."""
        # Check for specific disconnection errors that occur during operation
        is_disconnection_error = "Lost connection" in error_message or "sending" in error_message

        # Only trigger the full E-stop routine if we thought we were connected
        if is_disconnection_error and self.serial_handler.is_connected:
            self.log_message("! E-STOP DETECTED: Serial connection lost.", level="error")
            
            # 1. Save logs to a file
            self._save_logs_to_file()

            # 2. Exit ROS mode if it's active
            if self.ros_mode_var.get():
                self.log_message("Disabling ROS mode due to connection loss.")
                self.ros_mode_var.set(False)
                self._toggle_ros_mode() # This handles UI updates and thread cleanup

            # 3. Inform the user with a clear pop-up
            messagebox.showerror("Connection Lost", 
                                 "Serial connection lost (E-Stop detected).\n"
                                 "Logs have been saved.\n"
                                 "Please reconnect the device.", 
                                 parent=self.root)
            
            # 4. Force a disconnect in the handler to clean up internal state
            # This will also trigger the status_callback to update the GUI
            self.serial_handler.disconnect()
        else:
            # Handle other, more general serial errors
            if "Not connected" not in error_message: # Avoid spamming this specific error
                messagebox.showerror("Serial Error", error_message, parent=self.root)
                self.log_message(f"ERROR: {error_message}", level="error")

    def _toggle_connection_action(self): #
        if not self.serial_handler.is_connected: #
            port = self.port_entry.get() #
            self.serial_handler.connect(port) #
        else:
            self.serial_handler.disconnect() #
    
    def _update_control_mode_ui(self, event=None):
        is_degrees = self.control_mode_is_degrees_var.get()
        self.control_mode_button.config(text="MODE: DEGREES" if is_degrees else "MODE: STEPS")
        if is_degrees:
            self.individual_motor_toggle.config(state=tk.DISABLED)
            self.individual_motor_mode_var.set(False)
        else:
            self.individual_motor_toggle.config(state=tk.NORMAL)
        self._on_individual_motor_toggle()

    def _update_jaw_mode_ui(self, event=None):
        is_wrist_yaw_mode = self.wrist_yaw_mode_var.get()
        self.wrist_yaw_mode_button.config(text="MODE: WRIST_YAW" if is_wrist_yaw_mode else "MODE: JAWS")

        # ... (rest of the logic is unchanged)
        wrist_yaw_target_row = self.lj_target_row if hasattr(self, 'lj_target_row') else 3
        if hasattr(self, 'lj_label') and self.lj_label.winfo_exists():
            if not is_wrist_yaw_mode:
                self.lj_label.grid(row=self.lj_target_row, column=0, sticky="w", pady=2)
                self.lj_degrees_entry.grid(row=self.lj_target_row, column=1, pady=2)
            else:
                self.lj_label.grid_remove()
                self.lj_degrees_entry.grid_remove()
        if hasattr(self, 'rj_label') and self.rj_label.winfo_exists():
            if not is_wrist_yaw_mode:
                self.rj_label.grid(row=self.rj_target_row, column=0, sticky="w", pady=2)
                self.rj_degrees_entry.grid(row=self.rj_target_row, column=1, pady=2)
            else:
                self.rj_label.grid_remove()
                self.rj_degrees_entry.grid_remove()
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

    # --- Methods with unchanged logic from previous state ---
    def _on_individual_motor_toggle(self, event=None):
        is_individual = self.individual_motor_mode_var.get()
        is_degrees = self.control_mode_is_degrees_var.get()
        if is_individual and not is_degrees:
            self.q1_neg_button.config(state=tk.DISABLED)
            self.q2_neg_button.config(state=tk.DISABLED)
        else:
            self.q1_neg_button.config(state=tk.NORMAL)
            self.q2_neg_button.config(state=tk.NORMAL)
            
    def _toggle_ros_mode(self):
        is_ros_mode = self.ros_mode_var.get()
        if is_ros_mode:
            if not IS_ROS_AVAILABLE:
                messagebox.showerror("ROS Not Found", "The 'rospy' library is not installed. Please install ROS and its Python bindings to use this feature.", parent=self.root)
                self.ros_mode_var.set(False) # Revert the checkbox
                return
            self.ros_topic_entry.config(state=tk.NORMAL)
            self.ros_subscribe_button.config(state=tk.NORMAL)
            self.ros_freq_entry.config(state=tk.NORMAL) # Enable freq entry
            self.ros_status_var.set("Status: Ready")
            self.log_message("ROS mode enabled.")
        else:
            self._stop_ros_subscriber() # Stop any active subscription
            self.ros_topic_entry.config(state=tk.DISABLED)
            self.ros_subscribe_button.config(state=tk.DISABLED)
            self.ros_freq_entry.config(state=tk.DISABLED) # Disable freq entry
            self.ros_status_var.set("Status: Inactive")
            self.log_message("ROS mode disabled.")

    def _start_ros_subscriber(self):
        """Initializes ROS node if needed, then starts the subscriber thread."""
        if self.ros_thread and self.ros_thread.is_alive():
            self._stop_ros_subscriber()

        topic_name = self.ros_topic_var.get()
        if not topic_name:
            messagebox.showwarning("Input Error", "Please provide a ROS topic name.", parent=self.root)
            return

        # Initialize the ROS node in the main thread, only once.
        if not self.ros_node_initialized:
            try:
                self.log_message("Initializing ROS node for the first time...")
                # disable_signals=True is crucial to prevent the error in a Tkinter GUI
                rospy.init_node('elbow_gui_controller', anonymous=True, disable_signals=True)
                self.ros_node_initialized = True
                self.log_message("ROS node initialized successfully.")
            except Exception as e:
                self.log_message(f"Failed to initialize ROS node: {e}", level="error")
                self.ros_status_var.set("Status: Node Init Failed")
                return

        # Now that the node is initialized, start the subscriber thread
        self.ros_status_var.set(f"Status: Subscribing...")
        self.ros_thread = ROSSubscriberThread(topic_name, self.ros_queue, self.log_message)
        self.ros_thread.start()

    def _stop_ros_subscriber(self):
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.stop()
            self.ros_thread.join(timeout=2) # Wait up to 2 seconds for graceful shutdown
            if self.ros_thread.is_alive():
                self.log_message("ROS thread did not shut down gracefully.", level="warning")
            self.ros_thread = None
            self.ros_status_var.set("Status: Standby")
            self.log_message("ROS subscription stopped.")

    def _check_ros_queue(self):
        """Processes all messages in the queue for real-time control."""
        while not self.ros_queue.empty():
            try:
                target_positions = self.ros_queue.get_nowait()
                # self.log_message(f"ROS Command Received: Target {target_positions}")
                current_abs_positions = {
                    "EP": self.cumulative_ep_degrees_var.get(),
                    "EY": self.cumulative_ey_degrees_var.get(),
                    "WP": self.cumulative_wp_degrees_var.get(),
                    "LJ": self.cumulative_lj_degrees_var.get(),
                    "RJ": self.cumulative_rj_degrees_var.get(),
                }
                joint_degree_deltas = {
                    "EP": target_positions["Q1"] - current_abs_positions["EP"],
                    "EY": target_positions["Q2"] - current_abs_positions["EY"],
                    "WP": target_positions["Q3"] - current_abs_positions["WP"],
                    "LJ": target_positions["Q4L"] - current_abs_positions["LJ"],
                    "RJ": target_positions["Q4R"] - current_abs_positions["RJ"],
                }
                # self.log_message(f"Calculated Deltas: {joint_degree_deltas}")
                self._execute_degree_based_move(joint_degree_deltas)
            except queue.Empty:
                # This can happen in a race condition, it's safe to just break the loop
                break
            except Exception as e:
                self.log_message(f"Error processing ROS queue: {e}", level="error")
        
        # Schedule the next check
        self.root.after(self.ros_update_freq_ms.get(), self._check_ros_queue)

    def _joint_button_action(self, joint, sign):
        try:
            value = float(self.step_degree_input_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Invalid value. Please enter a number.", parent=self.root)
            return
        is_degrees = self.control_mode_is_degrees_var.get()
        is_individual = self.individual_motor_mode_var.get()
        if is_degrees:
            applied_value = value * sign
            joint_map = {"Q1": "EP", "Q2": "EY", "Q3": "WP", "Q4L": "LJ", "Q4R": "RJ"}
            joint_key = joint_map.get(joint)
            if joint_key:
                joint_degree_deltas = {joint_key: applied_value}
                self.log_message(f"Joint Move: {joint} by {applied_value}°")
                self._execute_degree_based_move(joint_degree_deltas)
        else:
            motor_steps = [0] * len(MotorIndex)
            if is_individual:
                motor_map = {
                    ("Q1", 1): MotorIndex.EP, ("Q2", 1): MotorIndex.EY,
                    ("Q3", 1): MotorIndex.WPD, ("Q3", -1): MotorIndex.WPU,
                    ("Q4L", 1): MotorIndex.LJR, ("Q4L", -1): MotorIndex.LJL,
                    ("Q4R", 1): MotorIndex.RJL, ("Q4R", -1): MotorIndex.RJR,
                }
                motor_to_move = motor_map.get((joint, sign))
                if motor_to_move is not None:
                    steps_value = int(value)
                    motor_steps[motor_to_move] = steps_value
                    self.log_message(f"Individual Motor Step: {motor_to_move.name} by {steps_value} steps", level="sent")
                else:
                     self.log_message(f"Invalid individual motor command: {joint} sign {sign}", level="warning")
                     return
            else:
                applied_value = value * sign
                steps = int(applied_value)
                if joint == "Q1": motor_steps[MotorIndex.EP] = steps
                elif joint == "Q2": motor_steps[MotorIndex.EY] = steps
                elif joint == "Q3":
                    motor_steps[MotorIndex.WPD] = steps; motor_steps[MotorIndex.WPU] = -steps
                elif joint == "Q4L":
                    motor_steps[MotorIndex.LJR] = steps; motor_steps[MotorIndex.LJL] = -steps
                elif joint == "Q4R":
                    motor_steps[MotorIndex.RJL] = steps; motor_steps[MotorIndex.RJR] = -steps
                self.log_message(f"Coordinated Step Move: {joint} by {steps} steps")
            steps_str = ",".join(map(str, motor_steps))
            cmd = f"MOVE_ALL_MOTORS:{steps_str}"
            self.serial_handler.send_command(cmd)
            self.log_message(f"Command: {cmd}", level="sent")

    def _send_update_limits_action(self):
        """Sends the UPDATE_LIMITS command and saves the new values."""
        try:
            t1_min = self.theta1_min_input_var.get()
            t1_max = self.theta1_max_input_var.get()
            t2_min = self.theta2_min_input_var.get()
            t2_max = self.theta2_max_input_var.get()
            
            int(t1_min); int(t1_max); int(t2_min); int(t2_max)

            cmd = f"UPDATE_LIMITS [{t1_min},{t1_max},{t2_min},{t2_max}]"
            
            # Send the command and only update/save if it succeeds
            if self.serial_handler.send_command(cmd):
                self.log_message(f"Command: {cmd}", level="sent")

                # Update the "Current" display labels
                self.theta1_min_display_var.set(t1_min)
                self.theta1_max_display_var.set(t1_max)
                self.theta2_min_display_var.set(t2_min)
                self.theta2_max_display_var.set(t2_max)
                
                # Save the new settings to the file immediately
                self._save_settings()
            else:
                 self.log_message("Failed to send UPDATE_LIMITS command.", level="error")

        except ValueError:
            messagebox.showerror("Input Error", "All limit values must be valid integers.", parent=self.root)
            self.log_message("Failed to send UPDATE_LIMITS: Invalid input.", level="error")


    def _send_find_limits_action(self):
        """Sends the FIND_LIMITS command."""
        cmd = "FIND_LIMITS"
        self.serial_handler.send_command(cmd)
        self.log_message(f"Command: {cmd}", level="sent")

    def _toggle_arduino_verbose_action(self):
        self.serial_handler.send_command("TOGGLE_VERBOSE")
        self.log_message("Command: TOGGLE_VERBOSE", level="sent")

    def _dedicated_move_by_degrees_action(self):
        try:
            if self.wrist_yaw_mode_var.get():
                jaw_val_for_yaw = float(self.wrist_yaw_degrees_var.get())
                joint_degree_deltas = {
                    "EP": float(self.ep_degrees_input_var.get()), "EY": float(self.ey_degrees_input_var.get()),
                    "WP": float(self.wp_degrees_input_var.get()), "LJ": jaw_val_for_yaw, "RJ": jaw_val_for_yaw,
                }
                self.wrist_yaw_degrees_var.set("0.0")
            else:
                joint_degree_deltas = {
                    "EP": float(self.ep_degrees_input_var.get()), "EY": float(self.ey_degrees_input_var.get()),
                    "WP": float(self.wp_degrees_input_var.get()), "LJ": float(self.lj_degrees_input_var.get()),
                    "RJ": float(self.rj_degrees_input_var.get()),
                }
                self.lj_degrees_input_var.set("0.0")
                self.rj_degrees_input_var.set("0.0")
            self.ep_degrees_input_var.set("0.0")
            self.ey_degrees_input_var.set("0.0")
            self.wp_degrees_input_var.set("0.0")
        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.", parent=self.root)
            return
        self.log_message(f"Dedicated Degree Input: Deltas {joint_degree_deltas}")
        self._execute_degree_based_move(joint_degree_deltas)

    def _execute_degree_based_move(self, joint_degree_deltas_input):
        all_joint_keys = ["EP", "EY", "WP", "LJ", "RJ"]
        full_joint_degree_deltas = {key: 0.0 for key in all_joint_keys}
        full_joint_degree_deltas.update(joint_degree_deltas_input)
        current_abs_positions = {
            "EP": self.cumulative_ep_degrees_var.get(), "EY": self.cumulative_ey_degrees_var.get(),
            "WP": self.cumulative_wp_degrees_var.get(), "LJ": self.cumulative_lj_degrees_var.get(),
            "RJ": self.cumulative_rj_degrees_var.get(),
        }
        joint_processors = {
            "EP": (current_abs_positions["EP"], full_joint_degree_deltas["EP"], q1_pl.get_steps, self.latest_dir["EP"]),
            "EY": (current_abs_positions["EY"], full_joint_degree_deltas["EY"], q2_pl.get_steps, self.latest_dir["EY"]),
            "WP": (current_abs_positions["WP"], full_joint_degree_deltas["WP"], q3_pl.get_steps, self.latest_dir["WP"]),
            "LJ": (current_abs_positions["LJ"], full_joint_degree_deltas["LJ"], q4_pl.get_steps_L, self.latest_dir["LJ"]),
            "RJ": (current_abs_positions["RJ"], full_joint_degree_deltas["RJ"], q4_pl.get_steps_R, self.latest_dir["RJ"]),
        }
        total_motor_steps = [0] * len(MotorIndex)
        for joint_name, (curr_theta, delta_theta, get_steps_function, latest_dir) in joint_processors.items():
            if delta_theta != 0:
                try:
                    # self.log_message(f"Calculating for {joint_name}: current={curr_theta:.2f}°, delta={delta_theta:.2f}°")
                    joint_specific_motor_steps, self.latest_dir[joint_name] = get_steps_function(curr_theta, delta_theta, latest_dir)
                    for motor_idx_enum in MotorIndex:
                        total_motor_steps[motor_idx_enum.value] += joint_specific_motor_steps[motor_idx_enum.value]
                    # self.log_message(f"  Steps from {joint_name}: {joint_specific_motor_steps}")
                except Exception as e:
                    self.log_message(f"  Error in {get_steps_function.__name__} for {joint_name}: {e}", level="error")
                    import traceback; self.log_message(traceback.format_exc(), level="error")
        final_integer_steps = [int(round(s)) for s in total_motor_steps]
        # self.log_message(f"Final Combined Steps: {final_integer_steps}")
        steps_str = ",".join(map(str, final_integer_steps))
        cmd = f"MOVE_ALL_MOTORS:{steps_str}"
        if self.serial_handler.send_command(cmd):
            # self.log_message(f"Command: {cmd}", level="sent")
            if full_joint_degree_deltas["EP"] != 0: self.cumulative_ep_degrees_var.set(round(current_abs_positions["EP"] + full_joint_degree_deltas["EP"], 2))
            if full_joint_degree_deltas["EY"] != 0: self.cumulative_ey_degrees_var.set(round(current_abs_positions["EY"] + full_joint_degree_deltas["EY"], 2))
            if full_joint_degree_deltas["WP"] != 0: self.cumulative_wp_degrees_var.set(round(current_abs_positions["WP"] + full_joint_degree_deltas["WP"], 2))
            if full_joint_degree_deltas["LJ"] != 0: self.cumulative_lj_degrees_var.set(round(current_abs_positions["LJ"] + full_joint_degree_deltas["LJ"], 2))
            if full_joint_degree_deltas["RJ"] != 0: self.cumulative_rj_degrees_var.set(round(current_abs_positions["RJ"] + full_joint_degree_deltas["RJ"], 2))
        else:
            self.log_message("Failed to send command.", level="error")

    def _reset_cumulative_degrees_display_action(self, from_test_mode=False, is_initial_setup=False):
        self.cumulative_ep_degrees_var.set(90.0)
        self.cumulative_ey_degrees_var.set(90.0)
        self.cumulative_wp_degrees_var.set(90.0)
        self.cumulative_lj_degrees_var.set(90.0)
        self.cumulative_rj_degrees_var.set(90.0)
        if not from_test_mode and not is_initial_setup:
            messagebox.showinfo("Reset", "Cumulative degree display has been reset.", parent=self.root)
        if not is_initial_setup:
            self.log_message("Cumulative degree display reset to reference positions.")

    def _reset_cumulative_degrees_display_from_test_mode(self):
        self._reset_cumulative_degrees_display_action(from_test_mode=True)
        self.log_message("Cumulative degrees display reset from 'Set Home'.")

    def cleanup_on_exit(self):
        self.log_message("Application exiting. Cleaning up...")
        self._save_settings()
        self._stop_ros_subscriber()
        if self.ros_node_initialized and IS_ROS_AVAILABLE and not rospy.is_shutdown():
            rospy.signal_shutdown("GUI is closing")
        self.serial_handler.cleanup()
        self.log_message("Cleanup complete. Goodbye.")