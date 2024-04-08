import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Define the system parameters
V_nom = 18.0  # volts
R = 8.4  # ohms
k_t = 0.042  # Nm/A
k_e = 0.042  # V/(rad/s)
b = 0.1e-6  # Nm/(rad/s)

# Given values for the inertia disc module
m_d = 0.053  # mass of the disk in kg
r_d = 0.0248  # radius of the disk in meters
m_h = 0.0106  # mass of the hub in kg
r_h = 0.0111  # radius of the hub in meters

# Calculating the moment of inertia of the disk and hub
J_d = 0.5 * m_d * r_d**2
J_h = 0.5 * m_h * r_h**2
J = J_d + J_h

# Time vector
t = np.linspace(0, 1, 1000)

# Define the test cases with their respective input voltages and data file names
test_cases = {
    "1V": "Data/RPM Test 1V.csv",
    "5V": "Data/RPM Test 5V.csv",
    "10V": "Data/RPM Test 10V.csv",
    "15V": "Data/RPM Test 15V.csv",
    "18V": "Data/RPM Test 18V.csv"
}

# Iterate over each test case
for test_name, data_file in test_cases.items():
    V_input = float(test_name[:-1])  # Extract voltage value from test name and convert to float
    
    # Recalculate based on current V_input
    A_mech = -b/J - k_e*k_t/(J*R)
    B_mech = k_t/(J*R)
    response_rad_s = -(B_mech / A_mech) * (1 - np.exp(A_mech * t)) * V_input
    response_rpm = response_rad_s * 60 / (2 * np.pi)
    
    # Read the CSV file
    data = pd.read_csv(data_file)
    
    # Find the time of the last 0 RPM and adjust the time column
    last_zero_time = data.loc[data['rpm'] == 0, 'time'].max() if not data[data['rpm'] == 0].empty else 0
    data['time'] -= last_zero_time  # Reset time so the last 0 RPM starts at t = 0s

    # Filter data for the first second after the first 0 RPM
    data_filtered = data[(data['time'] >= 0) & (data['time'] <= 1)]
    time_csv_filtered = data_filtered['time']
    rpm_csv_filtered = data_filtered['rpm']

    # Plot for each test case
    plt.figure(figsize=(10, 5))
    plt.plot(t, response_rpm, 'b-', label='Simulated RPM')
    plt.plot(time_csv_filtered, rpm_csv_filtered, 'r--', label='Tested RPM (CSV)')
    plt.xlim(0, 1)  # Ensure the plot is limited to 1 second after the first 0 RPM
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (RPM)')
    plt.title(f'RPM Test {test_name}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()