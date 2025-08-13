import math
import matplotlib.pyplot as plt
import numpy as np

# Make sure 'kinematic_model.py' is in the same directory as this script,
# so we can import the function.
from kinematic_model import l_pos_neg_py

def plot_cable_lengths():
    """
    Generates and displays a plot of the antagonistic cable lengths (l_pos, l_neg)
    for the Q1->Q3 coupling as the Q1 joint angle changes.
    """
    # Define the range of motion for Q1 in degrees.
    # The model is valid from -90 to +90 degrees.
    q1_angles_deg = np.linspace(-90, 90, 300) # 300 points for a smooth curve

    # Convert degrees to radians for the l_pos_neg_py function
    q1_angles_rad = np.radians(q1_angles_deg)

    # Lists to store the calculated cable lengths
    l_pos_values = []
    l_neg_values = []

    # Calculate the cable lengths for each angle in the range
    for angle_rad in q1_angles_rad:
        try:
            # We use j_num=3 for the Q1->Q3 coupling
            l_pos, l_neg = l_pos_neg_py(3, angle_rad)
            l_pos_values.append(l_pos * 1000) # Convert to mm for plotting
            l_neg_values.append(l_neg * 1000) # Convert to mm for plotting
        except Exception as e:
            print(f"An error occurred at angle {math.degrees(angle_rad)}: {e}")
            # Append None or a specific value if an error occurs to maintain list size
            l_pos_values.append(None)
            l_neg_values.append(None)


    # --- Plotting ---
    plt.figure(figsize=(12, 7))

    plt.plot(q1_angles_deg, l_pos_values, label='l_pos (Positive Cable)', color='blue')
    plt.plot(q1_angles_deg, l_neg_values, label='l_neg (Negative Cable)', color='red')

    # Adding labels and title for clarity
    plt.title('Q3 Cable Length vs. Q1 Joint Angle', fontsize=16)
    plt.xlabel('Q1 Joint Angle (degrees)', fontsize=12)
    plt.ylabel('Calculated Q3 Cable Length (mm)', fontsize=12)
    plt.grid(True)
    plt.legend(fontsize=10)
    
    # Show the plot
    plt.show()

if __name__ == '__main__':
    # This block will only run when the script is executed directly
    print("Generating plot for Q1->Q3 coupling model...")
    plot_cable_lengths()
    print("Plot window closed.")

