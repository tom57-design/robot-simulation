import pandas as pd
import matplotlib.pyplot as plt

# Specify the log file path
log_file_path = "datalog.log"  # Update with the actual path

# Specify the headers to plot (update as needed)
headers_to_plot = ["simTime_0", "basePos_0", "basePos_1", "basePos_2", "motors_pos_cur_0", "motors_vel_cur_0", "motors_tor_cur_0"]

# Load the log file
try:
    df = pd.read_csv(log_file_path)
    print("Log file loaded successfully.")
except Exception as e:
    print(f"Error loading log file: {e}")
    exit()

# Ensure specified headers exist in the file
missing_headers = [h for h in headers_to_plot if h not in df.columns]
if missing_headers:
    print(f"Warning: The following headers are missing from the log file: {missing_headers}")
    headers_to_plot = [h for h in headers_to_plot if h in df.columns]  # Remove missing headers

# Plot specified headers
if "simTime_0" in df.columns:
    time_column = df["simTime_0"]
    for header in headers_to_plot:
        if header != "simTime_0":  # Skip plotting time itself
            plt.figure()  # Create a new figure for each plot
            plt.plot(time_column, df[header], label=header)
            plt.xlabel("Time (s)")
            plt.ylabel(header)
            plt.title(f"Plot of {header}")
            plt.legend()
            plt.grid()
    plt.show()  # Show all plots at once after all are created
else:
    print("Error: 'simTime' column not found in the log file.")
