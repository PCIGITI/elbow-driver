# gui_test_motors_window.py
import tkinter as tk
from tkinter import ttk, messagebox
import config # For motor names, step counts

class TestMotorsWindow(tk.Toplevel):
    def __init__(self, parent, serial_handler, initial_selected_motor, log_to_main_callback):
        super().__init__(parent)
        self.parent = parent
        self.serial_handler = serial_handler
        self.log_to_main_callback = log_to_main_callback # To log "Exited Test Mode" etc.

        self.title("Test Motors Control")
        self.geometry(config.TEST_MOTORS_WINDOW_GEOMETRY)
        self.resizable(True, True)
        self.transient(parent) # Keep it above main window

        self.selected_motor_var = tk.StringVar(value=initial_selected_motor)
        self.motor_test_step_count_var = tk.IntVar(value=config.MOTOR_TEST_DEFAULT_STEP_COUNT)

        self.motor_buttons_in_window = {}

        self._create_widgets()
        self.highlight_selected_motor()

        self.protocol("WM_DELETE_WINDOW", self._on_close_window)
        self.grab_set() # Make it modal

        # Notify Arduino that test mode has started
        self.serial_handler.send_command("START_TEST_MOTORS")
        self.log_to_test_output("Sent: START_TEST_MOTORS")
        # Ensure Arduino is in sync with GUI's selected motor
        self.select_motor(self.selected_motor_var.get(), from_gui=True)


    def _create_widgets(self):
        content_frame = ttk.Frame(self, padding="10")
        content_frame.pack(expand=True, fill="both")

        # Left: Motor buttons
        motor_buttons_frame = ttk.Frame(content_frame)
        motor_buttons_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N)

        style = ttk.Style()
        style.configure("SelectedMotor.TButton", background="#0078D7", foreground="white", font=("Segoe UI", 9, "bold"))
        style.map("SelectedMotor.TButton", background=[('active', '#005a9e')], foreground=[('active', 'white')])
        style.configure("TButton", font=("Segoe UI", 9))

        for r, row_names in enumerate(config.MOTOR_NAMES_GROUPED):
            for c, name in enumerate(row_names):
                btn = ttk.Button(motor_buttons_frame, text=name, width=6,
                                 command=lambda n=name: self.select_motor(n, from_gui=True))
                btn.grid(row=r, column=c, pady=3, padx=3)
                self.motor_buttons_in_window[name] = btn

        # Middle: Controls for selected motor
        motor_controls_frame = ttk.Frame(content_frame)
        motor_controls_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N)

        ttk.Button(motor_controls_frame, text="Home Link (Set Home)", width=20,
                   command=self._home_link_command_action).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Fine Tension", width=20,
                   command=self._fine_tension_action).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Coarse Tension", width=20,
                   command=self._coarse_tension_action).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Detension", width=20,
                   command=self._detension_action).pack(pady=3, fill=tk.X)

        step_motor_manual_frame = ttk.Frame(motor_controls_frame)
        step_motor_manual_frame.pack(pady=3, fill=tk.X)
        ttk.Button(step_motor_manual_frame, text="Step Motor", width=15,
                   command=self._step_motor_by_amount_action).pack(side=tk.LEFT, pady=3)

        self.motor_test_step_slider_widget = ttk.Scale(
            step_motor_manual_frame, from_=config.MOTOR_TEST_STEP_MIN, to=config.MOTOR_TEST_STEP_MAX,
            orient=tk.HORIZONTAL, variable=self.motor_test_step_count_var,
            command=self._update_step_label, length=100)
        self.motor_test_step_slider_widget.pack(side=tk.LEFT, padx=5, pady=(3,0), fill=tk.X, expand=True)
        self.motor_test_step_display_label_widget = ttk.Label(
            step_motor_manual_frame, text=f"{self.motor_test_step_count_var.get()} steps", width=10)
        self.motor_test_step_display_label_widget.pack(side=tk.LEFT, padx=5, pady=(3,0))
        self._update_step_label() # Initialize

        ttk.Button(motor_controls_frame, text="Next Motor", width=20,
                   command=self._next_motor_action).pack(pady=3, fill=tk.X)
        ttk.Button(motor_controls_frame, text="Exit Test Mode", width=20,
                   command=self._on_close_window).pack(pady=3, fill=tk.X)

        # Right: Output area
        test_output_frame = ttk.Frame(content_frame)
        test_output_frame.grid(row=0, column=2, padx=10, pady=5, sticky=(tk.N, tk.S, tk.E, tk.W))
        content_frame.columnconfigure(2, weight=1)
        content_frame.rowconfigure(0, weight=1)

        ttk.Label(test_output_frame, text="Test Motor Log:").grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0,2))
        self.output_text_widget = tk.Text(test_output_frame, height=15, width=45)
        self.output_text_widget.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        test_scrollbar = ttk.Scrollbar(test_output_frame, orient=tk.VERTICAL, command=self.output_text_widget.yview)
        test_scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        test_output_frame.rowconfigure(1, weight=1)
        test_output_frame.columnconfigure(0, weight=1)
        self.output_text_widget['yscrollcommand'] = test_scrollbar.set

    def log_to_test_output(self, message):
        if self.output_text_widget and self.output_text_widget.winfo_exists():
            self.output_text_widget.insert(tk.END, message + "\n")
            self.output_text_widget.see(tk.END)

    def _update_step_label(self, event=None):
        if hasattr(self, 'motor_test_step_display_label_widget') and self.motor_test_step_display_label_widget.winfo_exists():
            steps = int(self.motor_test_step_count_var.get())
            self.motor_test_step_display_label_widget.config(text=f"{steps} steps")

    def highlight_selected_motor(self):
        current_selection = self.selected_motor_var.get()
        for name, btn in self.motor_buttons_in_window.items():
            if btn.winfo_exists():
                btn.config(style="SelectedMotor.TButton" if name == current_selection else "TButton")

    def select_motor(self, name, from_gui=False):
        self.selected_motor_var.set(name)
        self.highlight_selected_motor()
        if from_gui: # Only send command if selection originated from GUI
            self.serial_handler.send_command(f"SELECT_MOTOR:{name}")
            self.log_to_test_output(f"GUI selected: {name}. Sent SELECT_MOTOR:{name}")
        else: # Selection originated from Arduino (e.g. "NEXT_MOTOR" response)
            self.log_to_test_output(f"Arduino selected: {name}")


    def _home_link_command_action(self):
        self.serial_handler.send_command("SET_HOME")
        self.log_to_test_output("Sent: SET_HOME")
        # Notify main GUI to reset its cumulative display
        if hasattr(self.parent, '_reset_cumulative_degrees_display_from_test_mode'):
             self.parent._reset_cumulative_degrees_display_from_test_mode()
        messagebox.showinfo("Home Set", "SET_HOME command sent. Cumulative degrees in main window reset.", parent=self)


    def _fine_tension_action(self):
        self.serial_handler.send_command("FINE_TENSION")
        self.log_to_test_output(f"Sent: FINE_TENSION for {self.selected_motor_var.get()}")

    def _coarse_tension_action(self):
        self.serial_handler.send_command("COARSE_TENSION")
        self.log_to_test_output(f"Sent: COARSE_TENSION for {self.selected_motor_var.get()}")

    def _detension_action(self):
        self.serial_handler.send_command("DETENSION")
        self.log_to_test_output(f"Sent: DETENSION for {self.selected_motor_var.get()}")

    def _step_motor_by_amount_action(self):
        steps = int(self.motor_test_step_count_var.get())
        self.serial_handler.send_command(f"STEP_MOTOR_BY:{steps}")
        self.log_to_test_output(f"Sent: STEP_MOTOR_BY:{steps} for {self.selected_motor_var.get()}")

    def _next_motor_action(self):
        self.serial_handler.send_command("NEXT_MOTOR")
        self.log_to_test_output("Sent: NEXT_MOTOR") # Arduino will respond with new selection

    def _on_close_window(self):
        self.serial_handler.send_command("EXIT_TEST")
        if self.log_to_main_callback:
            self.log_to_main_callback("Exited Test Motors mode.")
        self.grab_release()
        self.destroy()
        # Notify main GUI that this window is closed
        if hasattr(self.parent, 'on_test_motors_window_closed'):
            self.parent.on_test_motors_window_closed()

    # This method will be called by the main GUI when Arduino sends TEST_MOTOR_SELECTED
    def update_selected_motor_from_arduino(self, motor_name):
        self.select_motor(motor_name, from_gui=False)