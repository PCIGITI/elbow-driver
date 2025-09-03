# main.py
import tkinter as tk
from tkinter import ttk
import time # For on_closing delay if needed

from gui_main_window import ElbowSimulatorGUI
from serial_handler import SerialHandler
import config # For app title or other global settings


def main_app():
    root = tk.Tk()

    # Attempt to set a modern theme
    style = ttk.Style(root)
    available_themes = style.theme_names()
    # Prefer clam, then vista, then others
    if "clam" in available_themes: style.theme_use("clam")
    elif "vista" in available_themes: style.theme_use("vista")
    elif "aqua" in available_themes: style.theme_use("aqua") # macOS
    # Add other preferred themes if desired

    # Initialize components
    serial_comms = SerialHandler() # Callbacks will be set by GUI

    app = ElbowSimulatorGUI(root, serial_handler=serial_comms)

    def on_closing_main_window():
        app.cleanup_on_exit() # Call the GUI's cleanup method
        # Add a small delay if needed for commands to send, though serial_handler.cleanup() should handle it.
        # time.sleep(0.1)
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing_main_window)
    root.mainloop()

if __name__ == "__main__":
    main_app()
