from QUBE import *
from logger import *
from com import *
from liveplot import *
import numpy as np
import control as ctrl
import threading
from time import time

# Initialize QUBE
port = "COM4"
baudrate = 115200
qube = QUBE(port, baudrate)
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

enableLogging()  # Enable or disable logging as needed
t_last = time()

pid = PID()

# Control targets
m_target = 900 # Target angle in degrees
p_target = 0  # Not used in this example
speed_target = 100  # Target speed in RPM (for SPEED_MODE)
Voltage = 0  # Initial control action

# System matrices
A = np.array([[0, 1], [0, -48824]])
B = np.array([[0], [294]])
D = np.array([[0], [0]])

# Control mode selection
ANGLE_MODE = 1
SPEED_MODE = 2
control_mode = ANGLE_MODE

# Adjust C matrix based on control mode
if control_mode == ANGLE_MODE:
    C = np.array([[1, 0]])  # Focus on angle for feedback
    m_target = m_target  # Define your angle setpoint here
elif control_mode == SPEED_MODE:
    C = np.array([[0, 1]])  # Focus on speed for feedback
    m_target = speed_target  # Use speed target for control

# Pole placement for state feedback
poles = np.array([-85, -100])
K = ctrl.place(A, B, poles)

def control(data, lock):
    global m_target, K, t_last, Voltage
    
    while True:
        qube.update()
        logdata = qube.getLogData(m_target, p_target, Voltage)
        save_data(logdata)

        dt = getDT()

        with lock:
            doMTStuff(data)

        current_angle = qube.getMotorAngle()
        current_speed = qube.getMotorRPM()

        # Choose the correct state based on control mode
        if control_mode == ANGLE_MODE:
            current_state = np.array([current_angle, 0])
            desired_state = np.array([m_target, 0])
        elif control_mode == SPEED_MODE:
            current_state = np.array([0, current_speed])
            desired_state = np.array([0, speed_target])
        
        state_error = current_state - desired_state
        control_action = -np.dot(K, state_error)
        Voltage = control_action / 10000
        qube.setMotorVoltage(Voltage)

def getDT():
    global t_last
    t_now = time()
    dt = t_now - t_last
    t_last += dt
    return dt

def doMTStuff(data):
    packet = data[7]
    pid.copy(packet.pid)
    if packet.resetEncoders:
        qube.resetMotorEncoder()
        qube.resetPendulumEncoder()
        packet.resetEncoders = False

    new_data = qube.getPlotData(m_target, p_target)
    for i, item in enumerate(new_data):
        data[i].append(item)

if __name__ == "__main__":
    _data = [[], [], [], [], [], [], [], Packet()]
    lock = threading.Lock()
    thread1 = threading.Thread(target=startPlot, args=(_data, lock))
    thread2 = threading.Thread(target=control, args=(_data, lock))
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
