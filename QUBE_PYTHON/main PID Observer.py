from time import time, sleep
import matplotlib.pyplot as plt
from QUBE import *
from logger import *
from com import *
from liveplot import *
import numpy as np
import control as ctrl
import threading

# System initialization
port, baudrate = "COM5", 115200
qube = QUBE(port, baudrate)
qube.resetMotorEncoder()
qube.resetPendulumEncoder()
enableLogging()

pid = PID()

# Control and observer configuration
m_target, s_target, Voltage = 0, 0, 0
target_change_delay, target_reached_threshold = 5, 10
SPEED_MODE = 2
ANGLE_MODE = 1
control_mode = SPEED_MODE

A = np.array([[0, 1, 0], [0, 1.3, 0], [-1, 0, 0]])
B = np.array([[0], [294], [0]])
C = np.array([[1, 0, 0]]) if control_mode == ANGLE_MODE else np.array([[0, 1, 0]])

poles = np.array([-0.4+0.42j, -0.4-0.42j, -4])
K = ctrl.place(A, B, poles)
L = ctrl.place(A.T, C.T, [-1, -1.1, -1.2]).T

# Data logs
time_log, angle_log, speed_log, voltage_log = [], [], [], []

def control():
    global m_target, s_target, Voltage, L, K
    last_target_change_time = time()
    target_reached = False
    x_hat = np.zeros((3, 1))
    t_last = time()

    while True:  # Add your stopping condition here
        dt = time() - t_last
        t_last += dt

        current_angle = qube.getMotorAngle()
        current_speed = qube.getMotorRPM()
        y = np.array([[current_angle if control_mode == ANGLE_MODE else current_speed]])
        u = np.array([[Voltage]])

        x_hat = A.dot(x_hat) + B.dot(u) + L.dot(y - C.dot(x_hat))  # Observer update
        
        # Control logic based on mode
        error = m_target - current_angle if control_mode == ANGLE_MODE else s_target - current_speed
        Voltage = -np.dot(K, x_hat)[0]  # Control law
        
        qube.setMotorVoltage(Voltage)
        
        # Logging for plotting
        time_log.append(t_last)
        angle_log.append(current_angle)
        speed_log.append(current_speed)
        voltage_log.append(Voltage)
        
        # Dynamic target adjustment
        if abs(error) <= target_reached_threshold:
            if not target_reached:
                target_reached = True
                last_target_change_time = time()
        else:
            target_reached = False

        if target_reached and (time() - last_target_change_time) >= target_change_delay:
            if control_mode == ANGLE_MODE:
                m_target += 10  # Adjust based on your requirement
            elif control_mode == SPEED_MODE:
                s_target += 100  # Adjust based on your requirement
            target_reached = False
        
        sleep(0.01)  # Adjust the sleep time based on your application

def plot_logs():
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time_log, angle_log, label='Motor Angle')
    plt.ylabel('Angle (degrees)')
    plt.subplot(3, 1, 2)
    plt.plot(time_log, speed_log, label='Motor Speed')
    plt.ylabel('Speed (RPM)')
    plt.subplot(3, 1, 3)
    plt.plot(time_log, voltage_log, label='Control Voltage')
    plt.xlabel('Time (s)')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.tight_layout()
    plt.show()

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
    plot_logs()
