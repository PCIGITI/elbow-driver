import tkinter as tk
from tkinter import ttk, messagebox
import math

import config # For constants, MotorIndex
from config import MotorIndex #
import q1_pl, q2_pl, q3_pl, q4_pl # Joint processors for step calculations

# ROS-related imports with a fallback if ROS is not installed
import queue
import threading
try:
    import rospy
    from std_msgs.msg import Float64MultiArray
    IS_ROS_AVAILABLE = True
except ImportError:
    IS_ROS_AVAILABLE = False
    
# --- Helper Class for ROS Communication ---

class ROSSubscriberThread(threading.Thread):
    """
    A dedicated thread to handle ROS node initialization and subscription
    to avoid blocking the main Tkinter GUI thread.
    """
    def __init__(self, topic_name, data_queue, log_callback):
        super().__init__(daemon=True)
        self._topic_name = topic_name
        self._data_queue = data_queue
        self._log_callback = log_callback
        self._subscriber = None
        self._is_running = True

    def _ros_callback(self, msg):
        """Callback function for the ROS subscriber."""
        # Expected message format: [NA, NA, Q1, Q2, Q3, Q4L] in radians
        if len(msg.data) >= 6:
            try:
                # Convert radians to degrees and create a dictionary of target positions
                target_positions_deg = {
                    "Q1": math.degrees(msg.data[2]), # EP
                    "Q2": math.degrees(msg.data[3]), # EY
                    "Q3": math.degrees(msg.data[4]), # WP
                    "Q4L": math.degrees(msg.data[5]), # LJ
                    "Q4R": -math.degrees(msg.data[5]) # RJ moves equal and opposite
                }
                # Put the processed data into the thread-safe queue for the GUI
                self._data_queue.put(target_positions_deg)
            except Exception as e:
                self._log_callback(f"ROS Callback Error: {e}")
        else:
            self._log_callback(f"ROS WARNING: Received message with insufficient data length: {len(msg.data)}")

    def run(self):
        """The main execution method of the thread."""
        self._log_callback("ROS Thread: Initializing node 'elbow_gui_controller'.")
        try:
            # Initialize the ROS node. anonymous=True ensures the node has a unique name.
            rospy.init_node('elbow_gui_controller', anonymous=True)
            # Create a subscriber for the specified topic
            self._subscriber = rospy.Subscriber(self._topic_name, Float64MultiArray, self._ros_callback)
            self._log_callback(f"ROS Thread: Subscribed to topic '{self._topic_name}'.")
            
            # rospy.spin() is a blocking call that keeps the script alive to receive messages.
            # We check our stop condition periodically.
            while self._is_running and not rospy.is_shutdown():
                rospy.sleep(0.1) # Sleep to reduce CPU usage

        except rospy.ROSInterruptException:
            self._log_callback("ROS Thread: Shutdown signal received.")
        except Exception as e:
            self._log_callback(f"ROS Thread: An error occurred: {e}")
        finally:
            self._log_callback("ROS Thread: Exiting.")

    def stop(self):
        """Signals the thread to stop."""
        self._log_callback("ROS Thread: Stop signal received.")
        self._is_running = False
        if self._subscriber:
            self._subscriber.unregister() # Cleanly unsubscribe


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
        self.ros_topic_var = tk.StringVar(value="/joint_commands") # Default topic
        self.ros_status_var = tk.StringVar(value="Status: Inactive")
        self.ros_queue = queue.Queue()
        self.ros_thread = None

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
        self._update_control_mode_ui() # Initialize new control UI
        self._check_ros_queue() # Start the loop to check for ROS messages

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

        # Row 1: NEW Joint Controls (Replaces D-Pads)
        self._create_joint_control_widgets(self.main_content_frame)

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

        # Row 3: ROS Control Frame
        self._create_ros_control_widgets(self.main_content_frame)

        # Row 4: Inline Positional Control Frame
        self._create_positional_control_frame_widgets(self.main_content_frame) #
    
    def _create_ros_control_widgets(self, parent_frame):
        """Creates the widgets for the ROS control feature."""
        ros_frame = ttk.LabelFrame(parent_frame, text="ROS Control", padding="10")
        ros_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=10)

        # Toggle button for ROS mode
        self.ros_mode_toggle = ttk.Checkbutton(
            ros_frame,
            text="ROS Mode",
            variable=self.ros_mode_var,
            command=self._toggle_ros_mode,
            style="Toolbutton"
        )
        self.ros_mode_toggle.grid(row=0, column=0, padx=5, pady=5)

        ttk.Label(ros_frame, text="Topic:").grid(row=0, column=1, padx=(10, 0), pady=5)
        self.ros_topic_entry = ttk.Entry(ros_frame, textvariable=self.ros_topic_var, width=30)
        self.ros_topic_entry.grid(row=0, column=2, padx=5, pady=5)

        self.ros_subscribe_button = ttk.Button(
            ros_frame, text="Subscribe", command=self._start_ros_subscriber
        )
        self.ros_subscribe_button.grid(row=0, column=3, padx=5, pady=5)

        self.ros_status_label = ttk.Label(ros_frame, textvariable=self.ros_status_var)
        self.ros_status_label.grid(row=0, column=4, padx=10, pady=5, sticky=tk.W)
        
        ros_frame.columnconfigure(4, weight=1) # Allow status label to expand

        # Initially disable ROS-specific widgets
        self.ros_topic_entry.config(state=tk.DISABLED)
        self.ros_subscribe_button.config(state=tk.DISABLED)

    def _toggle_ros_mode(self):
        """Handles enabling or disabling ROS mode."""
        is_ros_mode = self.ros_mode_var.get()
        
        if is_ros_mode:
            if not IS_ROS_AVAILABLE:
                messagebox.showerror("ROS Not Found", "The 'rospy' library is not installed. Please install ROS and its Python bindings to use this feature.", parent=self.root)
                self.ros_mode_var.set(False) # Revert the checkbox
                return

            self.ros_topic_entry.config(state=tk.NORMAL)
            self.ros_subscribe_button.config(state=tk.NORMAL)
            self.ros_status_var.set("Status: Ready to Subscribe")
            self.log_message("ROS mode enabled.")
            # Optionally, disable other manual controls to prevent conflicts
        else:
            self._stop_ros_subscriber() # Stop any active subscription
            self.ros_topic_entry.config(state=tk.DISABLED)
            self.ros_subscribe_button.config(state=tk.DISABLED)
            self.ros_status_var.set("Status: Inactive")
            self.log_message("ROS mode disabled.")
            
    def _start_ros_subscriber(self):
        """Starts the ROS subscriber thread."""
        if self.ros_thread and self.ros_thread.is_alive():
            self._stop_ros_subscriber() # Stop previous thread first

        topic_name = self.ros_topic_var.get()
        if not topic_name:
            messagebox.showwarning("Input Error", "Please provide a ROS topic name.", parent=self.root)
            return
        
        self.ros_status_var.set(f"Status: Subscribing to {topic_name}...")
        self.ros_thread = ROSSubscriberThread(topic_name, self.ros_queue, self.log_message)
        self.ros_thread.start()

    def _stop_ros_subscriber(self):
        """Stops the ROS subscriber thread if it's running."""
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.stop()
            self.ros_thread.join(timeout=1) # Wait for the thread to finish
            self.ros_thread = None
            self.ros_status_var.set("Status: Subscription stopped.")
            self.log_message("ROS subscription stopped.")

    def _check_ros_queue(self):
        """Periodically checks the queue for new messages from the ROS thread."""
        try:
            target_positions = self.ros_queue.get_nowait()
            self.log_message(f"ROS Command Received: Target Positions {target_positions}")
            
            # Get current absolute positions from the GUI
            current_abs_positions = {
                "EP": self.cumulative_ep_degrees_var.get(),
                "EY": self.cumulative_ey_degrees_var.get(),
                "WP": self.cumulative_wp_degrees_var.get(),
                "LJ": self.cumulative_lj_degrees_var.get(),
                "RJ": self.cumulative_rj_degrees_var.get(),
            }

            # Calculate the delta needed to reach the target from the current position
            joint_degree_deltas = {
                "EP": target_positions["Q1"] - current_abs_positions["EP"],
                "EY": target_positions["Q2"] - current_abs_positions["EY"],
                "WP": target_positions["Q3"] - current_abs_positions["WP"],
                "LJ": target_positions["Q4L"] - current_abs_positions["LJ"],
                "RJ": target_positions["Q4R"] - current_abs_positions["RJ"],
            }
            
            self.log_message(f"Calculated Deltas for Move: {joint_degree_deltas}")
            self._execute_degree_based_move(joint_degree_deltas)

        except queue.Empty:
            # No new message, which is the normal case
            pass
        finally:
            # Schedule this method to run again after 100ms
            self.root.after(100, self._check_ros_queue)

    def _create_joint_control_widgets(self, parent_frame):
        """Creates the new joint control UI as requested."""
        joint_super_frame = ttk.LabelFrame(parent_frame, text="Joint Controls", padding="10")
        joint_super_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=5)
        joint_super_frame.columnconfigure(0, weight=1) # Center the content

        # --- Top row: Mode, Input, and Individual Toggle ---
        top_controls_frame = ttk.Frame(joint_super_frame)
        top_controls_frame.pack(pady=5, padx=5)

        self.control_mode_button = ttk.Checkbutton(
            top_controls_frame,
            text="Mode: Degrees",
            variable=self.control_mode_is_degrees_var,
            command=self._update_control_mode_ui,
            style="Toolbutton"
        )
        self.control_mode_button.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(top_controls_frame, text="Value:").pack(side=tk.LEFT, padx=(0, 5))
        self.step_degree_entry = ttk.Entry(top_controls_frame, textvariable=self.step_degree_input_var, width=10)
        self.step_degree_entry.pack(side=tk.LEFT, padx=(0, 15))

        self.individual_motor_toggle = ttk.Checkbutton(
            top_controls_frame,
            text="Individual Motor Control",
            variable=self.individual_motor_mode_var,
            style="Toolbutton",
            command=self._on_individual_motor_toggle
        )
        self.individual_motor_toggle.pack(side=tk.LEFT, padx=(0, 10))

        # --- Joint Buttons ---
        joints_frame = ttk.Frame(joint_super_frame, padding="5")
        joints_frame.pack(pady=(10,0))

        # Define joint data
        joint_data = [
            ("Q1 (Elbow Pitch)", "Q1"),
            ("Q2 (Elbow Yaw)", "Q2"),
            ("Q3 (Wrist Pitch)", "Q3"),
        ]

        # Create Q1, Q2, Q3 rows
        for i, (label, joint_id) in enumerate(joint_data):
            neg_btn = ttk.Button(joints_frame, text="-", width=4, command=lambda j=joint_id: self._joint_button_action(j, -1))
            neg_btn.grid(row=i, column=0, padx=5, pady=2)
            ttk.Label(joints_frame, text=label, width=20, anchor="center").grid(row=i, column=1)
            ttk.Button(joints_frame, text="+", width=4, command=lambda j=joint_id: self._joint_button_action(j, 1)).grid(row=i, column=2, padx=5, pady=2)
            
            if joint_id == "Q1": self.q1_neg_button = neg_btn
            if joint_id == "Q2": self.q2_neg_button = neg_btn

        # Q4 Frame for Jaws
        q4_frame = ttk.Frame(joints_frame)
        q4_frame.grid(row=len(joint_data), column=0, columnspan=3, pady=5)

        # Q4L
        ttk.Button(q4_frame, text="-", width=4, command=lambda: self._joint_button_action("Q4L", -1)).grid(row=0, column=0)
        ttk.Label(q4_frame, text="Q4L", anchor="center").grid(row=0, column=1, padx=(2, 8))
        ttk.Button(q4_frame, text="+", width=4, command=lambda: self._joint_button_action("Q4L", 1)).grid(row=0, column=2)

        # Spacer
        ttk.Label(q4_frame, text=" (Jaws) ", anchor="center").grid(row=0, column=3, padx=15)

        # Q4R - Note the button order from the prompt
        ttk.Button(q4_frame, text="+", width=4, command=lambda: self._joint_button_action("Q4R", 1)).grid(row=0, column=4)
        ttk.Label(q4_frame, text="Q4R", anchor="center").grid(row=0, column=5, padx=(2, 8))
        ttk.Button(q4_frame, text="-", width=4, command=lambda: self._joint_button_action("Q4R", -1)).grid(row=0, column=6)

    def _create_positional_control_frame_widgets(self, parent_frame): #
        positional_super_frame = ttk.LabelFrame(parent_frame, text="Positional Control (Dedicated Input Fields)", padding="10") #
        positional_super_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10) #

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
        
        self.lj_target_row = row_idx
        self.rj_target_row = row_idx + 1 

        self.wrist_yaw_label = ttk.Label(input_frame, text="Wrist Yaw (°):")
        self.wrist_yaw_entry = ttk.Entry(input_frame, textvariable=self.wrist_yaw_degrees_var, width=10)

        self.lj_label = ttk.Label(input_frame, text="Left Jaw (°):") #
        self.lj_degrees_entry = ttk.Entry(input_frame, textvariable=self.lj_degrees_input_var, width=10) #
        self.lj_label.grid(row=self.lj_target_row, column=0, sticky="w", pady=2) #
        self.lj_degrees_entry.grid(row=self.lj_target_row, column=1, pady=2) #
        row_idx +=1 

        self.rj_label = ttk.Label(input_frame, text="Right Jaw (°):") #
        self.rj_degrees_entry = ttk.Entry(input_frame, textvariable=self.rj_degrees_input_var, width=10) #
        self.rj_label.grid(row=self.rj_target_row, column=0, sticky="w", pady=2) #
        self.rj_degrees_entry.grid(row=self.rj_target_row, column=1, pady=2) #
        row_idx +=1 


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
        output_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5) #
        self.main_content_frame.grid_rowconfigure(5, weight=1) #
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

    def _update_control_mode_ui(self, event=None):
        """Updates UI elements based on the control mode (Degrees/Steps)."""
        is_degrees = self.control_mode_is_degrees_var.get()

        if is_degrees:
            self.control_mode_button.config(text="Mode: Degrees")
            # Disable individual motor control in degrees mode
            self.individual_motor_toggle.config(state=tk.DISABLED)
            self.individual_motor_mode_var.set(False) # Uncheck it
        else: # Steps mode
            self.control_mode_button.config(text="Mode: Steps")
            # Enable individual motor control
            self.individual_motor_toggle.config(state=tk.NORMAL)

        # Update button states based on the new mode
        self._on_individual_motor_toggle()

    def _on_individual_motor_toggle(self, event=None):
        """Updates button states when individual motor mode is toggled or control mode changes."""
        is_individual = self.individual_motor_mode_var.get()
        is_degrees = self.control_mode_is_degrees_var.get()

        # The toggle should only have an effect if we are in steps mode.
        if is_individual and not is_degrees:
            # Gray out Q1 and Q2 negative buttons
            self.q1_neg_button.config(state=tk.DISABLED)
            self.q2_neg_button.config(state=tk.DISABLED)
        else:
            # Re-enable them for coordinated or degree mode
            self.q1_neg_button.config(state=tk.NORMAL)
            self.q2_neg_button.config(state=tk.NORMAL)

    def _update_jaw_mode_ui(self, event=None):
        """Updates the UI elements for Jaw/Wrist Yaw mode for dedicated degree inputs."""
        is_wrist_yaw_mode = self.wrist_yaw_mode_var.get()

        if hasattr(self, 'wrist_yaw_mode_button') and self.wrist_yaw_mode_button.winfo_exists():
            self.wrist_yaw_mode_button.config(text="Wrist Yaw Mode" if is_wrist_yaw_mode else "Jaws Mode")

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

    def _joint_button_action(self, joint, sign):
        """Handles all joint control button presses."""
        try:
            value = float(self.step_degree_input_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Invalid value. Please enter a number.", parent=self.root)
            return

        is_degrees = self.control_mode_is_degrees_var.get()
        is_individual = self.individual_motor_mode_var.get()

        if is_degrees:
            # --- Degree-Based Mode (always coordinated) ---
            # Signs multiply to get the final degree value.
            applied_value = value * sign
            joint_map = {"Q1": "EP", "Q2": "EY", "Q3": "WP", "Q4L": "LJ", "Q4R": "RJ"}
            joint_key = joint_map.get(joint)
            if joint_key:
                joint_degree_deltas = {joint_key: applied_value}
                self.log_message(f"Joint Move: {joint} by {applied_value}°")
                self._execute_degree_based_move(joint_degree_deltas)
        else:
            # --- Step-Based Mode ---
            motor_steps = [0] * len(MotorIndex)
            
            if is_individual:
                # --- Individual Motor Control ---
                # Buttons select the motor, value box provides the signed step count.
                motor_map = {
                    ("Q1", 1): MotorIndex.EP,
                    ("Q2", 1): MotorIndex.EY,
                    ("Q3", 1): MotorIndex.WPD,
                    ("Q3", -1): MotorIndex.WPU,
                    ("Q4L", 1): MotorIndex.LJR,
                    ("Q4L", -1): MotorIndex.LJL,
                    ("Q4R", 1): MotorIndex.RJL,
                    ("Q4R", -1): MotorIndex.RJR,
                }
                motor_to_move = motor_map.get((joint, sign))
                if motor_to_move is not None:
                    steps_value = int(value) # Use the value from the box directly
                    motor_steps[motor_to_move] = steps_value
                    self.log_message(f"Individual Motor Step: {motor_to_move.name} by {steps_value} steps")
                else:
                     self.log_message(f"Invalid individual motor command: {joint} sign {sign}")
                     return
            else:
                # --- Coordinated Step Mode ---
                # Signs multiply to get the final step value.
                applied_value = value * sign
                steps = int(applied_value)
                if joint == "Q1": motor_steps[MotorIndex.EP] = steps
                elif joint == "Q2": motor_steps[MotorIndex.EY] = steps
                elif joint == "Q3": # Wrist Pitch
                    motor_steps[MotorIndex.WPD] = steps
                    motor_steps[MotorIndex.WPU] = -steps
                elif joint == "Q4L": # Left Jaw Yaw
                    motor_steps[MotorIndex.LJR] = steps
                    motor_steps[MotorIndex.LJL] = -steps
                elif joint == "Q4R": # Right Jaw Yaw
                    motor_steps[MotorIndex.RJL] = steps
                    motor_steps[MotorIndex.RJR] = -steps
                
                self.log_message(f"Coordinated Step Move: {joint} by {steps} steps")

            steps_str = ",".join(map(str, motor_steps))
            cmd = f"MOVE_ALL_MOTORS:{steps_str}"
            self.serial_handler.send_command(cmd)
            self.log_message(f"Sent Step-Based Move: {cmd}")

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

            self.ep_degrees_input_var.set("0.0") #
            self.ey_degrees_input_var.set("0.0") #
            self.wp_degrees_input_var.set("0.0") #

        except ValueError:
            messagebox.showerror("Input Error", "Invalid degree value. Please enter numbers only.", parent=self.root) #
            return
        
        self.log_message(f"Dedicated Degree Input: Deltas {joint_degree_deltas}") #
        self._execute_degree_based_move(joint_degree_deltas) #

    def _execute_degree_based_move(self, joint_degree_deltas_input): #
        """
        Core logic to calculate and send motor commands based on degree deltas.
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
        self._stop_ros_subscriber() # Ensure ROS thread is stopped cleanly
        self.serial_handler.cleanup() #
        self.log_message("Serial handler cleaned up.") #

