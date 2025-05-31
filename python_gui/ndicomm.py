### aurora_tracking.py in pyNDI library : a sample aurora tracking for pyNDI
### ONOGI, Shinya, PhD, Department of Biomedical Information, Institute of Biomaterials and Bioengineering
### Tokyo Medical and Dental University

import time
from pyNDI.aurora import *
import keyboard
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from datetime import datetime

# Create Data directory if it doesn't exist
if not os.path.exists('Data'):
    os.makedirs('Data')

# Find next trial number
existing_trials = [f for f in os.listdir('Data') if f.startswith('trial_')]
if existing_trials:
    next_trial = max([int(f.split('_')[1].split('.')[0]) for f in existing_trials]) + 1
else:
    next_trial = 1

# Initialize data storage
start_time = None
times = []
positions = {'x': [], 'y': [], 'z': []}

try:
    t = aurora()
    t.connect('COM5')
    t.command(RESET())
    t.initialize()
    t.activate_wired_tools()
    t.start_tracking()
    
    print("Collecting data... Press 'escape' to stop")
    
    while not keyboard.is_pressed('escape'):
        data, stat = t.update()
        
        # Get positions for both tools
        tool10_pos = None
        tool11_pos = None
        
        for k, v in data.items():
            if v.status == handle_data.Valid:
                if k == '10':
                    tool10_pos = v.transformation_data.translation
                elif k == '11':
                    tool11_pos = v.transformation_data.translation
        
        # If both tools are valid, calculate relative position
        if tool10_pos is not None and tool11_pos is not None:
            if start_time is None:
                start_time = time.time()
            
            current_time = time.time() - start_time
            times.append(current_time)
            
            # Calculate relative position
            rel_x = tool10_pos[0] - tool11_pos[0]
            rel_y = tool10_pos[1] - tool11_pos[1]
            rel_z = tool10_pos[2] - tool11_pos[2]
            
            positions['x'].append(rel_x)
            positions['y'].append(rel_y)
            positions['z'].append(rel_z)
            
            print(f"Time: {current_time:.2f}s - Relative position: X={rel_x:.2f}, Y={rel_y:.2f}, Z={rel_z:.2f}")
        
        time.sleep(0.05)

    print("\nSaving data and creating plots...")
    
    # Create and save the plots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
    fig.suptitle('Tool 10 Position Relative to Tool 11')
    
    # Plot the data
    ax1.plot(times, positions['x'], 'b-', label='X')
    ax2.plot(times, positions['y'], 'g-', label='Y')
    ax3.plot(times, positions['z'], 'r-', label='Z')
    
    # Configure subplots
    for ax, title in zip([ax1, ax2, ax3], ['X Position', 'Y Position', 'Z Position']):
        ax.set_title(title)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (mm)')
        ax.grid(True)
        ax.legend()
    
    plt.tight_layout()
    
    # Save the plot
    plt.savefig(f'Data/trial_{next_trial}.png')
    plt.close()
    
    # Save the data to CSV
    df = pd.DataFrame({
        'Time': times,
        'X': positions['x'],
        'Y': positions['y'],
        'Z': positions['z']
    })
    df.to_csv(f'Data/trial_{next_trial}.csv', index=False)
    
    print(f"Data saved to Data/trial_{next_trial}.csv")
    print(f"Plot saved to Data/trial_{next_trial}.png")
    
    t.stop_tracking()
    t.command(COMM(0, 0, 0, 0, 0))
    
except Exception as e:
    print(f"An error occurred: {e}")
    pass