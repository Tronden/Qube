import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import place_poles

# System parameters from the Quanser QUBE-Servo 2 manual
g = 9.81  # m/s^2, gravitational acceleration
J = 0.5 * 0.053 * (0.0248 ** 2)  # Adjusted based on the disk's mass and radius from manual
b = 0.1e-6  # Assumed damping coefficient, needs adjustment based on manual data
k_t = 0.042  # Torque constant in Nm/A, based on manual
k_e = 0.042  # Back-EMF constant in V/(rad/s), based on manual
R = 8.4  # Terminal resistance in ohms, based on manual

# Adjusting the linearized model to reflect a control system scenario more accurately
A = np.array([[0, 1], [-k_t*k_e/(J*R), -b/J]])
B = np.array([[0], [k_t/(J*R)]])

# Desired pole locations, chosen to ensure system stability
p_d = np.array([-2, -3])  # Distinct and suitable for control

# Calculate controller gain using pole placement
K = place_poles(A, B, p_d).gain_matrix

# Simulation parameters
dt = 0.01  # s, timestep (100 Hz)
simTime = 4  # s, total simulation time
N = int(simTime/dt)  # number of simulation steps

# Initial state
x_0 = np.array([20, 0])*np.pi/180  # Initial condition [theta, omega], converted to radians

# Memory allocation
u = np.zeros(N)
x = np.zeros((2, N+1))
x[:, 0] = x_0


for i in range(N):
    u[i] = -K.dot(x[:, i])
    next_step = A.dot(x[:, i]) + B.dot([u[i]])  # Add an extra dimension to u[i]
    x[:, i+1] = x[:, i] + dt * next_step.flatten()  # Use flatten() to ensure correct shape


# Plotting in a single figure window
plt.figure(figsize=(19, 9))  # Adjust size as needed for fullscreen effect

plt.suptitle('QUBE-Servo 2 System Response and Control Input', fontsize=16)

# System response plot
plt.subplot(2, 1, 1)
plt.plot(np.linspace(0, simTime, N+1), x[0, :], label='Theta (rad)')
plt.plot(np.linspace(0, simTime, N+1), x[1, :], label='Omega (rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('State')
plt.legend()
plt.grid(True)
plt.title('System Response')

# Control input plot
plt.subplot(2, 1, 2)
plt.plot(np.linspace(0, simTime, N), u, color='green', label='Control Input (u)')
plt.xlabel('Time (s)')
plt.ylabel('Input Voltage (V)')
plt.legend()
plt.grid(True)
plt.title('Control Input Over Time')

plt.get_current_fig_manager().window.state('zoomed')
plt.tight_layout()
plt.subplots_adjust(top=0.9)
plt.show()  