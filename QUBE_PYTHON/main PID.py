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
m_target = 0 # Target angle in degrees
s_target = 0 # Target speed in RPM
Voltage = 0  # Initial control action

target_change_delay = 15
last_target_change_time = time()
target_reached = False
target_reached_threshold = 10 # degrees or rpm

# Control mode selection
ANGLE_MODE = 1
SPEED_MODE = 2
control_mode = ANGLE_MODE

# Original system matrices
A = np.array([[0, 1, 0],
              [0, 1.3, 0],
              [-1, 0, 0]])
B = np.array([[0], [294], [0]])
C = np.array([[1, 0, 0]]) if control_mode == ANGLE_MODE else np.array([[0, 1, 0]])

poles = np.array([-0.4+0.42j, -0.4-0.42j, -4])
K = ctrl.place(A, B, poles)

def control(data, lock):
    global K, m_target, s_target, t_last, Voltage, target_reached
    
    integral_error = 0

    while True:
        qube.update()
        logdata = qube.getLogData(m_target, s_target, Voltage)
        save_data(logdata)

        dt = getDT()

        with lock:
            doMTStuff(data)

        current_angle = qube.getMotorAngle()
        current_speed = qube.getMotorRPM()

        if control_mode == ANGLE_MODE:
            s_target = 0
            Value = 100
        elif control_mode == SPEED_MODE:
            m_target = current_angle
            Value = 1000

        Pos_Error = m_target - current_angle
        Speed_Error = s_target - current_speed
        
        # Update the integral of the error
        integral_error += -((Pos_Error + Speed_Error) * dt)
        integral_error = max(min(integral_error, Value), -Value)  # Limit the integral error
        Voltage = K[0][0] * Pos_Error + K[0][1] * Speed_Error + K[0][2] * integral_error
        print(K[0][0] * Pos_Error, K[0][1] * Speed_Error, K[0][2] * integral_error, Voltage)
        if control_mode == ANGLE_MODE:
            # Check if target is reached within threshold
            if abs(Pos_Error) <= target_reached_threshold:
                if not target_reached:
                    target_reached = True
                    last_target_change_time = time()
            else:
                target_reached = False

        elif control_mode == SPEED_MODE:
            # Check if target is reached within threshold
            if abs(Speed_Error) <= target_reached_threshold:
                if not target_reached:
                    target_reached = True
                    last_target_change_time = time()
            else:
                target_reached = False

        # Change target after delay if target is reached
        if target_reached and (time() - last_target_change_time) >= target_change_delay:
            if control_mode == ANGLE_MODE:
                m_target += 6000  # Define new_target_angle as needed
            elif control_mode == SPEED_MODE:
                s_target += 500 # Target speed in RPM (for SPEED_MODE)
            target_reached = False  # Reset to wait for new target to be reached

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

    new_data = qube.getPlotData(m_target, s_target)
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