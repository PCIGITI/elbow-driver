import numpy as np
import os

# --- Configuration ---
POLYNOMIAL_DEGREE = 4
# Get the absolute path of the directory containing this script.
_script_dir = os.path.dirname(os.path.abspath(__file__))

# Build absolute paths to the data files. This is the most reliable method.
DATA_FILEPATH_Q1_Q3 = os.path.join(_script_dir, 'q1q3-angles-29-aug.txt')
DATA_FILEPATH_Q2_Q4 = os.path.join(_script_dir, 'q2q4-angles-29-aug.txt')
# NEW: Filepath for the second-layer compensation data
DATA_FILEPATH_Q2_Q4_COMP = os.path.join(_script_dir, 'q2-q4-comp.txt')


# --- Global Model Variables (initialized on first use) ---
_model_q1_q3 = None
_model_q2_q4 = None
# NEW: Model for the second-layer compensation
_model_q2_q4_comp = None


def _remove_outliers_iqr(x_data, y_data):
    """
    Removes statistical outliers from a dataset based on the IQR of the x-data.
    """
    Q1 = np.percentile(x_data, 25)
    Q3 = np.percentile(x_data, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    mask = (x_data >= lower_bound) & (x_data <= upper_bound)
    return x_data[mask], y_data[mask]

def _clean_data(x_data, y_data, x_valid_range=None, y_valid_range=None):
    """
    Performs a multi-step cleaning process:
    1. Filters data by the valid operational range for x (if specified).
    2. Filters data by the valid operational range for y (if specified).
    3. Removes statistical outliers from the already filtered data.
    """
    # Make copies to avoid modifying original data passed to the function
    x_filtered, y_filtered = np.copy(x_data), np.copy(y_data)

    # Step 1: Filter by valid operational range for x
    if x_valid_range is not None:
        initial_count = len(x_filtered)
        lower_x, upper_x = x_valid_range
        mask = (x_filtered >= lower_x) & (x_filtered <= upper_x)
        x_filtered, y_filtered = x_filtered[mask], y_filtered[mask]
        num_removed = initial_count - len(x_filtered)
        if num_removed > 0:
            print(f"Data Cleaning (X Range): Removed {num_removed} points outside the valid x-range {x_valid_range}.")

    # Step 2: Filter by valid operational range for y
    if y_valid_range is not None:
        initial_count = len(x_filtered)
        lower_y, upper_y = y_valid_range
        mask = (y_filtered >= lower_y) & (y_filtered <= upper_y)
        x_filtered, y_filtered = x_filtered[mask], y_filtered[mask]
        num_removed = initial_count - len(x_filtered)
        if num_removed > 0:
            print(f"Data Cleaning (Y Range): Removed {num_removed} points outside the valid y-range {y_valid_range}.")

    # Step 3: Apply statistical outlier removal on the ranged data
    initial_count = len(x_filtered)
    x_clean, y_clean = _remove_outliers_iqr(x_filtered, y_filtered)
    num_removed_iqr = initial_count - len(x_clean)
    if num_removed_iqr > 0:
         print(f"Data Cleaning (IQR): Removed {num_removed_iqr} statistical outliers from the valid range.")

    return x_clean, y_clean

def _initialize_q1_q3_model():
    """
    Internal function to build the Q1->Q3 polynomial model on cleaned data.
    """
    global _model_q1_q3
    try:
        data = np.loadtxt(DATA_FILEPATH_Q1_Q3, delimiter=',')
        q1_data, q3_data = data[:, 0], data[:, 1]
        q1_clean, q3_clean = _clean_data(q1_data, q3_data, x_valid_range=(0, 3))
        coeffs = np.polyfit(q1_clean, q3_clean, POLYNOMIAL_DEGREE)
        _model_q1_q3 = np.poly1d(coeffs)
        print("Q1->Q3 coupling model initialized successfully on cleaned data.")
    except Exception as e:
        print(f"CRITICAL ERROR: Could not initialize Q1->Q3 model from '{DATA_FILEPATH_Q1_Q3}'. Error: {e}")
        _model_q1_q3 = None

def _initialize_q2_q4_model():
    """
    Internal function to build the Q2->Q4 polynomial model on cleaned data.
    """
    global _model_q2_q4
    try:
        data = np.loadtxt(DATA_FILEPATH_Q2_Q4, delimiter=',')
        q2_data, q4_data = data[:, 0], data[:, 1]
        q2_clean, q4_clean = _clean_data(q2_data, q4_data, x_valid_range=(0, 3), y_valid_range=(0, 4))
        coeffs = np.polyfit(q2_clean, q4_clean, POLYNOMIAL_DEGREE)
        _model_q2_q4 = np.poly1d(coeffs)
        print("Q2->Q4 coupling model (Layer 1) initialized successfully on cleaned data.")
    except FileNotFoundError:
        print(f"CRITICAL ERROR: Data file not found for Q2->Q4 model at '{DATA_FILEPATH_Q2_Q4}'.")
        _model_q2_q4 = None
    except Exception as e:
        print(f"CRITICAL ERROR: Could not initialize Q2->Q4 model from '{DATA_FILEPATH_Q2_Q4}'. Error: {e}")
        _model_q2_q4 = None

def _initialize_q2_q4_comp_model():
    """
    NEW: Internal function to build the second-layer Q2->Q4 compensation model.
    """
    global _model_q2_q4_comp
    try:
        data = np.loadtxt(DATA_FILEPATH_Q2_Q4_COMP, delimiter=',')
        q2_data, q4_residual_data = data[:, 0], data[:, 1]
        
        # Clean the compensation data. Based on the provided file, the valid ranges are different.
        q2_clean, q4_clean = _clean_data(q2_data, q4_residual_data, x_valid_range=(0, 3), y_valid_range=(0, 3.0))

        coeffs = np.polyfit(q2_clean, q4_clean, POLYNOMIAL_DEGREE)
        _model_q2_q4_comp = np.poly1d(coeffs)
        print("Q2->Q4 compensation model (Layer 2) initialized successfully.")
    except FileNotFoundError:
        print(f"CRITICAL ERROR: Data file not found for Q2->Q4 compensation model at '{DATA_FILEPATH_Q2_Q4_COMP}'.")
        _model_q2_q4_comp = None
    except Exception as e:
        print(f"CRITICAL ERROR: Could not initialize Q2->Q4 compensation model. Error: {e}")
        _model_q2_q4_comp = None

def get_q3_change(current_q1: float, delta_q1: float) -> float:
    """
    Calculates the expected change in Q3 for a given movement of Q1.
    """
    if _model_q1_q3 is None:
        _initialize_q1_q3_model()
        if _model_q1_q3 is None:
            return 0.0

    if delta_q1 == 0:
        return 0.0
            
    q1_initial = current_q1
    q1_final = current_q1 + delta_q1

    q3_initial_predicted = _model_q1_q3(q1_initial)
    q3_final_predicted = _model_q1_q3(q1_final)
    
    return q3_final_predicted - q3_initial_predicted

def get_q4_change(current_q2: float, delta_q2: float) -> float:
    """
    MODIFIED: Calculates the expected change in Q4 for a given movement of Q2,
    now including a two-layer compensation model.
    """
    # Ensure the first-layer model is initialized
    if _model_q2_q4 is None:
        _initialize_q2_q4_model()
        if _model_q2_q4 is None:
            return 0.0 # Cannot proceed if the primary model fails

    # Ensure the second-layer compensation model is initialized
    if _model_q2_q4_comp is None:
        _initialize_q2_q4_comp_model()

    if delta_q2 == 0:
        return 0.0
            
    q2_initial = current_q2
    q2_final = current_q2 + delta_q2

    # --- Layer 1 Compensation ---
    q4_initial_predicted_L1 = _model_q2_q4(q2_initial)
    q4_final_predicted_L1 = _model_q2_q4(q2_final)
    delta_q4_L1 = q4_final_predicted_L1 - q4_initial_predicted_L1
    
    # --- Layer 2 Compensation (only if the model loaded successfully) ---
    delta_q4_L2 = 0.0
    if _model_q2_q4_comp is not None:
        q4_initial_predicted_L2 = _model_q2_q4_comp(q2_initial)
        q4_final_predicted_L2 = _model_q2_q4_comp(q2_final)
        delta_q4_L2 = q4_final_predicted_L2 - q4_initial_predicted_L2

    # --- Combine the compensations ---
    total_delta_q4 = delta_q4_L1 + delta_q4_L2
    
    return total_delta_q4

# --- Optional: For testing and visualization ---
def visualize_fit(model_choice='q1_q3'):
    """
    A helper function to plot data against a model to verify the fit.
    """
    import matplotlib.pyplot as plt

    if model_choice == 'q1_q3':
        if _model_q1_q3 is None: _initialize_q1_q3_model()
        if _model_q1_q3 is None: return
        model_to_plot, filepath = _model_q1_q3, DATA_FILEPATH_Q1_Q3
        x_label, y_label = "Q1 Position (radians)", "Q3 Position (radians)"
        title = "Model Fit for Q1 vs. Q3 (Multi-Step Cleaning)"
        x_valid_range, y_valid_range = (0, 3), None

    elif model_choice == 'q2_q4':
        if _model_q2_q4 is None: _initialize_q2_q4_model()
        if _model_q2_q4 is None: return
        model_to_plot, filepath = _model_q2_q4, DATA_FILEPATH_Q2_Q4
        x_label, y_label = "Q2 Position (radians)", "Q4 Position (radians)"
        title = "Model Fit for Q2 vs. Q4 (Layer 1)"
        x_valid_range, y_valid_range = (0, 3), (0, 4)

    # NEW: Visualization for the second-layer compensation model
    elif model_choice == 'q2_q4_comp':
        if _model_q2_q4_comp is None: _initialize_q2_q4_comp_model()
        if _model_q2_q4_comp is None: return
        model_to_plot, filepath = _model_q2_q4_comp, DATA_FILEPATH_Q2_Q4_COMP
        x_label, y_label = "Q2 Position (radians)", "Residual Q4 Position (radians)"
        title = "Model Fit for Q2 vs. Residual Q4 (Layer 2 Compensation)"
        x_valid_range, y_valid_range = (0, 3), (0, 3.0)

    else:
        print(f"Invalid model_choice: '{model_choice}'. Use 'q1_q3', 'q2_q4', or 'q2_q4_comp'.")
        return

    # Load the ORIGINAL raw data to show all points
    data = np.loadtxt(filepath, delimiter=',')
    x_data, y_data = data[:, 0], data[:, 1]

    plt.figure(figsize=(10, 8))
    plt.scatter(x_data, y_data, label='Experimental Data (All Points)', color='black', s=15, alpha=0.5)

    # Clean the data to get the correct plotting range
    x_clean, _ = _clean_data(x_data, y_data, x_valid_range=x_valid_range, y_valid_range=y_valid_range)
    
    x_range = np.linspace(x_clean.min(), x_clean.max(), 300)
    plt.plot(x_range, model_to_plot(x_range), label=f'Averaged Model (Degree {POLYNOMIAL_DEGREE})', color='red', linewidth=2)
    
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.grid(True)
    plt.legend()
    plt.show()

# --- Main execution block for direct testing of this script ---
if __name__ == '__main__':
    print("--- Testing Q1->Q3 Model ---")
    predicted_change_1 = get_q3_change(current_q1=2.0, delta_q1=-0.5)
    print(f"Movement from 2.0 to 1.5 -> Predicted delta_q3: {predicted_change_1:.6f}")
    
    print("\n--- Testing Q2->Q4 Model (Two-Layer) ---")
    predicted_change_q4 = get_q4_change(current_q2=1.0, delta_q2=-0.5)
    if _model_q2_q4 is not None:
         print(f"Movement from 1.0 to 0.5 -> Predicted total delta_q4: {predicted_change_q4:.6f}")

    # To see the plots, uncomment the lines below
    # visualize_fit(model_choice='q1_q3')
    # visualize_fit(model_choice='q2_q4')
    visualize_fit(model_choice='q2_q4_comp') # Visualize the fit for the new compensation model