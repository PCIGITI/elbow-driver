import tkinter as tk
from tkinter import ttk, messagebox
import math

class ElbowSimulatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Elbow Control Simulator")
        self.root.geometry("600x400")
        
        # Initialize variables
        self.current_ep_angle = 0.0  # Current Elbow Pitch angle
        self.current_ey_angle = 0.0  # Current Elbow Yaw angle
        self.is_verbose = False
        
        # Create main frame
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create widgets
        self.create_widgets()
        
        # Create output area
        self.create_output_area()
    
    def create_widgets(self):
        # Status frame
        status_frame = ttk.LabelFrame(self.main_frame, text="Current Status", padding="5")
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Current angles display
        self.ep_angle_label = ttk.Label(status_frame, text="Elbow Pitch: 0.0°")
        self.ep_angle_label.grid(row=0, column=0, padx=5)
        
        self.ey_angle_label = ttk.Label(status_frame, text="Elbow Yaw: 0.0°")
        self.ey_angle_label.grid(row=0, column=1, padx=5)
        
        # Control frame
        control_frame = ttk.LabelFrame(self.main_frame, text="Movement Controls", padding="5")
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Relative movement controls
        ttk.Label(control_frame, text="Relative Movement:").grid(row=0, column=0, columnspan=2, pady=5)
        
        # Elbow Pitch Relative
        ttk.Label(control_frame, text="Elbow Pitch:").grid(row=1, column=0, padx=5)
        self.ep_rel_entry = ttk.Entry(control_frame, width=10)
        self.ep_rel_entry.grid(row=1, column=1, padx=5)
        ttk.Button(control_frame, text="Move EP Rel", 
                  command=self.move_ep_relative).grid(row=1, column=2, padx=5)
        
        # Elbow Yaw Relative
        ttk.Label(control_frame, text="Elbow Yaw:").grid(row=2, column=0, padx=5)
        self.ey_rel_entry = ttk.Entry(control_frame, width=10)
        self.ey_rel_entry.grid(row=2, column=1, padx=5)
        ttk.Button(control_frame, text="Move EY Rel", 
                  command=self.move_ey_relative).grid(row=2, column=2, padx=5)
        
        # Absolute movement controls
        ttk.Label(control_frame, text="Absolute Movement:").grid(row=3, column=0, columnspan=2, pady=5)
        
        # Elbow Pitch Absolute
        ttk.Label(control_frame, text="Elbow Pitch:").grid(row=4, column=0, padx=5)
        self.ep_abs_entry = ttk.Entry(control_frame, width=10)
        self.ep_abs_entry.grid(row=4, column=1, padx=5)
        ttk.Button(control_frame, text="Move EP Abs", 
                  command=self.move_ep_absolute).grid(row=4, column=2, padx=5)
        
        # Elbow Yaw Absolute
        ttk.Label(control_frame, text="Elbow Yaw:").grid(row=5, column=0, padx=5)
        self.ey_abs_entry = ttk.Entry(control_frame, width=10)
        self.ey_abs_entry.grid(row=5, column=1, padx=5)
        ttk.Button(control_frame, text="Move EY Abs", 
                  command=self.move_ey_absolute).grid(row=5, column=2, padx=5)
        
        # Test and Verbose controls
        test_frame = ttk.Frame(self.main_frame)
        test_frame.grid(row=2, column=0, columnspan=2, pady=5)
        
        ttk.Button(test_frame, text="Start Elbow Test", 
                  command=self.start_elbow_test).grid(row=0, column=0, padx=5)
        ttk.Button(test_frame, text="Set Verbose ON", 
                  command=lambda: self.set_verbose(True)).grid(row=0, column=1, padx=5)
        ttk.Button(test_frame, text="Set Verbose OFF", 
                  command=lambda: self.set_verbose(False)).grid(row=0, column=2, padx=5)
    
    def create_output_area(self):
        # Output frame
        output_frame = ttk.LabelFrame(self.main_frame, text="Output", padding="5")
        output_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        # Output text area
        self.output_text = tk.Text(output_frame, height=8, width=60)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Scrollbar for output
        scrollbar = ttk.Scrollbar(output_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.output_text['yscrollcommand'] = scrollbar.set
    
    def log_message(self, message):
        self.output_text.insert(tk.END, message + "\n")
        self.output_text.see(tk.END)
    
    def move_ep_relative(self):
        try:
            angle = float(self.ep_rel_entry.get())
            self.current_ep_angle += angle
            self.ep_angle_label.config(text=f"Elbow Pitch: {self.current_ep_angle:.1f}°")
            self.log_message(f"Moving Elbow Pitch relative by {angle}°")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for the angle")
    
    def move_ey_relative(self):
        try:
            angle = float(self.ey_rel_entry.get())
            self.current_ey_angle += angle
            self.ey_angle_label.config(text=f"Elbow Yaw: {self.current_ey_angle:.1f}°")
            self.log_message(f"Moving Elbow Yaw relative by {angle}°")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for the angle")
    
    def move_ep_absolute(self):
        try:
            angle = float(self.ep_abs_entry.get())
            self.current_ep_angle = angle
            self.ep_angle_label.config(text=f"Elbow Pitch: {self.current_ep_angle:.1f}°")
            self.log_message(f"Moving Elbow Pitch to absolute position {angle}°")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for the angle")
    
    def move_ey_absolute(self):
        try:
            angle = float(self.ey_abs_entry.get())
            self.current_ey_angle = angle
            self.ey_angle_label.config(text=f"Elbow Yaw: {self.current_ey_angle:.1f}°")
            self.log_message(f"Moving Elbow Yaw to absolute position {angle}°")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for the angle")
    
    def start_elbow_test(self):
        self.log_message("Starting elbow test sequence...")
        # Simulate the test sequence from main.cpp
        test_angles = [0, 45, 90, 135, 180, 135, 90, 45, 0]
        for angle in test_angles:
            self.log_message(f"Testing position: {angle}°")
    
    def set_verbose(self, state):
        self.is_verbose = state
        self.log_message(f"Verbose mode {'enabled' if state else 'disabled'}")

def main():
    root = tk.Tk()
    app = ElbowSimulatorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
