from QUBE import *
from logger import *
from com import *
from liveplot import *
import numpy as np
import control as ctrl
import threading
from time import time

# Initialize QUBE
port = "COM5"
baudrate = 115200
qube = QUBE(port, baudrate)
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

enableLogging()  # Enable or disable logging as needed
t_last = time()

pid = PID()

# Control targets
m_target = 1000 # Target angle in degrees
p_target = 0  # Not used in this example
s_target = 1000 # Target speed in RPM (for SPEED_MODE)
Voltage = 0  # Initial control action

# Original system matrices
A = np.array([[0, 1, 0],
              [0, -48824, 0],
              [-1, 0, 0]])

B = np.array([[0], [294], [0]])

# Control mode selection
ANGLE_MODE = 1
SPEED_MODE = 2
control_mode = SPEED_MODE

# Adjust C matrix based on control mode
if control_mode == ANGLE_MODE:
    C = np.array([[1, 0, 0]])  # Focus on angle for feedback
elif control_mode == SPEED_MODE:
    C = np.array([[0, 1, 0]])  # Focus on speed for feedback

poles = np.array([-5-8.92j, -5+8.92j, -50])
K = ctrl.place(A, B, poles)

def control(data, lock):
    global m_target, K, t_last, Voltage
    
    integral_error = 0

    while True:
        qube.update()
        logdata = qube.getLogData(m_target, p_target, Voltage)
        save_data(logdata)

        dt = getDT()

        with lock:
            doMTStuff(data)

        current_angle = qube.getMotorAngle()
        current_speed = qube.getMotorRPM()

        if control_mode == ANGLE_MODE:
            Pos_Target = m_target
            Speed_Target = 0
        elif control_mode == SPEED_MODE:
            Pos_Target = current_angle
            Speed_Target = s_target

        Pos_Error = Pos_Target - current_angle
        Speed_Error = Speed_Target - current_speed
        
        # Update the integral of the error
        integral_error += (Pos_Error * dt / 100) + (Speed_Error * dt / 100)
        integral_error = max(min(integral_error, 100), -100)  # Limit the integral error

        current_state = np.array([current_angle, current_speed, integral_error])  
        desired_state = np.array([Pos_Target, Speed_Target, 0])
        state_error = current_state - desired_state
        print(K)
        Voltage = -np.dot(K, state_error) / 1000
        print(Voltage)
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