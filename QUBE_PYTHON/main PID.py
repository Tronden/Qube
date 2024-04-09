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

enableLogging()  
t_last = time()

m_target = 90 #Angle
p_target = 0
s_target = 2000 #RPM
Voltage = 0
integral_error = 0

# Control mode selection
ANGLE_MODE = 1
SPEED_MODE = 2
control_mode = ANGLE_MODE

K1 = 0.02
K2 = 0.0165
K3 = 0.00177

def control(data, lock):
    global m_target, s_target, K1, K2, K3, t_last, Voltage
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
        integral_error += (Pos_Error * dt) + (Speed_Error * dt)
        integral_error = max(min(integral_error, 100), -100)  # Limit the integral error

        Voltage = K1 * Pos_Error + K2 * Speed_Error + K3 * integral_error
        qube.setMotorVoltage(Voltage)
        
def getDT():
    global t_last
    t_now = time()
    dt = t_now - t_last
    t_last = t_now  # Update the last time stamp for the next iteration
    return dt

def doMTStuff(data):
    # Assuming packet and pid are handled correctly elsewhere
    packet = data[7]
    # pid.copy(packet.pid)
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