import time
from pyNDI.aurora import * # Imports aurora, RESET, COMM, handle_data, etc.
import keyboard
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# --- Plot Setup ---
plt.ion()  # Interactive mode ON
fig = plt.figure(figsize=(10, 8)) # Adjust figure size as needed
ax = fig.add_subplot(111, projection='3d')

# Initialize points for two tools (handles 10 and 11, based on your previous output)
# Initial positions are NaN so they don't plot until valid data is received.
# Using integer keys for handles as they likely appear in `data.items()`.
initial_nan_position = np.array([np.nan, np.nan, np.nan])

tool_plots = {
    10: ax.scatter([initial_nan_position[0]], [initial_nan_position[1]], [initial_nan_position[2]], s=100, c='red', marker='o', label='Tool 10'),
    11: ax.scatter([initial_nan_position[0]], [initial_nan_position[1]], [initial_nan_position[2]], s=100, c='blue', marker='^', label='Tool 11')
}

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title('Live Aurora Tool Tracking')
ax.legend()

# Set initial plot limits (adjust these based on your expected working volume)
# Given your Z data is around -150mm, and X/Y around 50-70mm.
# Let's set a broader range initially.
AXIS_LIMIT_XY = 200  # +/- 200 mm for X and Y
AXIS_LIMIT_Z_NEG = -400 # mm
AXIS_LIMIT_Z_POS = 100  # mm

ax.set_xlim([-AXIS_LIMIT_XY, AXIS_LIMIT_XY])
ax.set_ylim([-AXIS_LIMIT_XY, AXIS_LIMIT_XY])
ax.set_zlim([AXIS_LIMIT_Z_NEG, AXIS_LIMIT_Z_POS]) # Adjusted for typical negative Z for Aurora

# --- Main Tracking Logic ---
# Store current positions of tools, using integer keys
current_positions = {
    10: np.array([np.nan, np.nan, np.nan]),
    11: np.array([np.nan, np.nan, np.nan])
}

try:
    t = aurora()
    print("Connecting to Aurora...")
    # !!! IMPORTANT: Verify your COM port is correct !!!
    t.connect('COM5') # For example, 'COM3', 'COM4', etc.
    # t.connect('/dev/ttyS0') # Example for Linux

    print("Sending RESET command to Aurora...")
    t.command(RESET())
    print("Initializing Aurora system...")
    t.initialize()
    print("Activating wired tools...")
    t.activate_wired_tools() # This function in pyNDI should handle PHSR, PINIT, PENA
    print("Starting tracking...")
    t.start_tracking()
    print("Tracking started. Press 'Escape' to stop.")
    print("Plot window should appear. Ensure it has focus if 'Esc' doesn't work in terminal.")


    while not keyboard.is_pressed('escape'):
        data, stat = t.update()  # Get data from Aurora device

        processed_handles_in_current_frame = set()

        for tool_handle_key, tool_data_obj in data.items():
            # Ensure the key is an integer if your pyNDI library returns it as such
            try:
                tool_handle_int = int(tool_handle_key)
            except ValueError:
                print(f"Warning: Could not convert tool handle '{tool_handle_key}' to int. Skipping.")
                continue

            processed_handles_in_current_frame.add(tool_handle_int)

            if tool_handle_int in current_positions:  # Check if it's one of the tools we want to plot
                if tool_data_obj.status == handle_data.Valid:
                    # Assuming transformation_data.translation is an accessible structure (like a list, tuple, or an object with x,y,z)
                    translation = tool_data_obj.transformation_data.translation
                    if hasattr(translation, 'x') and hasattr(translation, 'y') and hasattr(translation, 'z'):
                        current_positions[tool_handle_int] = np.array([translation.x, translation.y, translation.z])
                    elif isinstance(translation, (list, tuple)) and len(translation) == 3:
                        current_positions[tool_handle_int] = np.array(translation)
                    else:
                        # Fallback or error if the structure is unexpected
                        current_positions[tool_handle_int] = np.array([np.nan, np.nan, np.nan])
                        # print(f"Tool {tool_handle_int}: Unexpected translation data format: {translation}")

                    # Optional: print to console as before
                    # print(f"Tool {tool_handle_int}: {current_positions[tool_handle_int]}")

                else: # Missing or Disabled
                    current_positions[tool_handle_int] = np.array([np.nan, np.nan, np.nan]) # Mark as NaN so it won't plot
                    # Optional: print to console
                    # status_name = "Unknown"
                    # if tool_data_obj.status == handle_data.Missing: status_name = "Missing"
                    # elif tool_data_obj.status == handle_data.Disabled: status_name = "Disabled"
                    # print(f"Tool {tool_handle_int}: {status_name}")


        # For any of our target tools that were NOT in the current data packet, mark them as NaN
        for target_handle in current_positions.keys():
            if target_handle not in processed_handles_in_current_frame:
                current_positions[target_handle] = np.array([np.nan, np.nan, np.nan])

        # Update plot data
        for handle_int_key, plot_object in tool_plots.items():
            position_to_plot = current_positions.get(handle_int_key, initial_nan_position) # Default to NaN if somehow key is missing
            # Matplotlib's _offsets3d expects a tuple of arrays/lists: ([x_coords], [y_coords], [z_coords])
            plot_object._offsets3d = ([position_to_plot[0]], [position_to_plot[1]], [position_to_plot[2]])

        fig.canvas.draw_idle()  # Redraw the plot
        plt.pause(0.01)  # Crucial for updating plot in interactive mode and processing GUI events.
                         # Adjust if too slow or too fast. 0.05 was original sleep.

    print("Stopping tracking...")
    t.stop_tracking()
    print("Resetting communication parameters...")
    t.command(COMM(0, 0, 0, 0, 0)) # Reset to default comms
    print("Script finished.")

except keyboard.بلافاصلهKeyboardInterrupt: # If Ctrl+C is pressed
    print("\nKeyboard interrupt detected. Exiting...")
except Exception as e:
    print(f"An error occurred: {e}")
    import traceback
    traceback.print_exc() # Print full traceback for debugging
finally:
    print("Cleaning up...")
    if 't' in locals() and hasattr(t, 'is_connected') and t.is_connected(): # t.is_connected() is hypothetical
        try:
            print("Attempting to stop tracking and reset comms due to exit/error...")
            t.stop_tracking()
            t.command(COMM(0, 0, 0, 0, 0))
        except Exception as e_cleanup:
            print(f"Error during cleanup: {e_cleanup}")

    plt.ioff() # Interactive mode OFF
    if 'fig' in locals():
        print("Close the plot window to fully exit if it doesn't close automatically.")
        plt.show() # Keep plot open until manually closed, or use plt.close(fig)
        # plt.close(fig) # Uncomment to close plot automatically on script end/error
    print("Plot closed or script ended.")