import tkinter as tk
from tkinter import ttk

class TestMotorsWindow:
    def __init__(self, parent, motor_names_grouped, motor_test_step_min, motor_test_step_max,
                 callbacks):
        self.window = tk.Toplevel(parent)
        self.window.title("Test Motors Control")
        self.window.geometry("750x450")
        self.window.resizable(True, True)
        self.window.transient(parent)
        
        self.motor_buttons = {}
        self.motor_test_step_count_var = tk.IntVar(value=0)
        self.callbacks = callbacks
        
        # Create main content
        content_frame = ttk.Frame(self.window, padding="10")
        content_frame.pack(expand=True, fill="both")
        
        self._create_motor_buttons(content_frame, motor_names_grouped)
        self._create_control_panel(content_frame, motor_test_step_min, motor_test_step_max)
        self._create_output_area(content_frame)
        
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
        self.window.grab_set()

    def _create_motor_buttons(self, parent, motor_names_grouped):
        motor_buttons_frame = ttk.Frame(parent)
        motor_buttons_frame.grid(row=0, column=0, padx=10, pady=5, sticky=tk.N)
        
        style = ttk.Style()
        style.configure("SelectedMotor.TButton", background="#0078D7", foreground="white", 
                       font=("Segoe UI", 9, "bold"))
        style.map("SelectedMotor.TButton", background=[('active', '#005a9e')], 
                 foreground=[('active', 'white')])
        style.configure("TButton", font=("Segoe UI", 9))

        for r, row_names in enumerate(motor_names_grouped):
            for c, name in enumerate(row_names):
                btn = ttk.Button(motor_buttons_frame, text=name, width=6,
                               command=lambda n=name: self.callbacks["select_motor"](n))
                btn.grid(row=r, column=c, pady=3, padx=3)
                self.motor_buttons[name] = btn

    def _create_control_panel(self, parent, step_min, step_max):
        controls_frame = ttk.Frame(parent)
        controls_frame.grid(row=0, column=1, padx=10, pady=5, sticky=tk.N)

        buttons = [
            ("Home Link (Set Home)", self.callbacks["home_link"]),
            ("Fine Tension", self.callbacks["fine_tension"]),
            ("Coarse Tension", self.callbacks["coarse_tension"]),
            ("Detension", self.callbacks["detension"])
        ]

        for text, command in buttons:
            ttk.Button(controls_frame, text=text, width=20, command=command).pack(pady=3, fill=tk.X)

        self._create_step_control(controls_frame, step_min, step_max)
        
        ttk.Button(controls_frame, text="Next Motor", width=20, 
                  command=self.callbacks["next_motor"]).pack(pady=3, fill=tk.X)
        ttk.Button(controls_frame, text="Exit Test Mode", width=20,
                  command=self.callbacks["exit"]).pack(pady=3, fill=tk.X)

    def _create_step_control(self, parent, step_min, step_max):
        step_frame = ttk.Frame(parent)
        step_frame.pack(pady=3, fill=tk.X)
        
        ttk.Button(step_frame, text="Step Motor", width=15,
                  command=self.callbacks["step_motor"]).pack(side=tk.LEFT, pady=3)
        
        self.step_slider = ttk.Scale(
            step_frame, from_=step_min, to=step_max, orient=tk.HORIZONTAL,
            variable=self.motor_test_step_count_var, 
            command=self._update_step_label, length=100)
        self.step_slider.pack(side=tk.LEFT, padx=5, pady=(3,0), fill=tk.X, expand=True)
        
        self.step_label = ttk.Label(step_frame, text="0 steps", width=10)
        self.step_label.pack(side=tk.LEFT, padx=5, pady=(3,0))

    def _create_output_area(self, parent):
        output_frame = ttk.Frame(parent)
        output_frame.grid(row=0, column=2, padx=10, pady=5, sticky=(tk.N, tk.S, tk.E, tk.W))
        parent.columnconfigure(2, weight=1)
        parent.rowconfigure(0, weight=1)

        ttk.Label(output_frame, text="Test Motor Log:").grid(row=0, column=0, 
                                                          columnspan=2, sticky=tk.W, pady=(0,2))
        
        self.output_text = tk.Text(output_frame, height=15, width=45)
        self.output_text.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, 
                                command=self.output_text.yview)
        scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        
        output_frame.rowconfigure(1, weight=1)
        output_frame.columnconfigure(0, weight=1)
        self.output_text['yscrollcommand'] = scrollbar.set

    def _update_step_label(self, *args):
        steps = self.motor_test_step_count_var.get()
        self.step_label.config(text=f"{steps} steps")

    def highlight_selected_motor(self, selected_name):
        for name, btn in self.motor_buttons.items():
            if btn.winfo_exists():
                btn.config(style="SelectedMotor.TButton" if name == selected_name else "TButton")

    def log_output(self, message):
        if self.output_text and self.output_text.winfo_exists():
            self.output_text.insert(tk.END, message + "\n")
            self.output_text.see(tk.END)

    def get_step_amount(self):
        try:
            return self.motor_test_step_count_var.get()
        except tk.TclError:
            return 0

    def on_close(self):
        self.callbacks["exit"]()

    def close(self):
        if self.window and self.window.winfo_exists():
            self.window.grab_release()
            self.window.destroy()
