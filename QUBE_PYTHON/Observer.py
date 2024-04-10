#Hentet litt kode fra https://hackaday.io/project/175294/files?fbclid=IwAR0jCo5pFQu7vpAGqGHUPpqMjJQj8zbowYHmuBuW1W0Crmcir9w37C6BFp0

import matplotlib.pyplot as plt
import control
import control.matlab
import numpy as np

Aa = np.matrix([[0,  1, 0],
                [0, 1.3, 0],
                [1, 0, 0]])
Ba = np.matrix([[0],[294],[0]])
Br = np.matrix([[0],[0],[-1]])
Ca = np.matrix([[1, 0, 0]])

Tstep = 0.1

poles = [-0.4+0.66j, -0.4-0.66j, -4]
Kfeedback = control.place(Aa, Ba, poles)
feedbackA = Aa - Ba@Kfeedback

A = np.matrix( [[0, 1],[0, 1.3]] )
B =  np.matrix( [[0],[294]] )
C =  np.matrix( [[1, 0]] )

sys = control.ss(A,B,C,0)
sysd = control.matlab.c2d(sys, Tstep)

Vd = np.matrix([[3, 0],   
                [0, 3]])
Vn = 10

Kobs,P,E = control.lqe(A, Vd, C, Vd, Vn)

print("Kalman gain matrix:")
print(Kobs)

print("\nEigenvalues:")
print(E)

kalmanObs = control.ss(A - Kobs @ sys.C, np.concatenate( (B, Kobs), axis=1), np.eye(2), 0)

kalmanObsd = control.matlab.c2d(kalmanObs,Tstep)
print("Kalman matrices:\n",kalmanObsd)

def doKalmanSim(kalmanGain, noise):
    kalmanObs = control.ss(A - kalmanGain @ sys.C, np.concatenate( (B, kalmanGain), axis=1), np.eye(2), 0 )
    kalmanObsd = control.matlab.c2d(kalmanObs,Tstep)

    obs_U = np.zeros((2,1))
    xhat = np.matrix([[0],[0]])

    x= np.matrix([[200], [0]])

    xdesired = np.zeros(250)
    xdesired[75:150]= 200 
    xArray = np.matrix([[0], [0], [0]])
    xhatArray = np.matrix([[0], [0]])
    uArray = np.matrix([0]).reshape(1,1)
    measuredArray = [0]

    sigma = 0
    u = np.matrix(0)

    for i in range(250):
        measuredX = x[0,0] + np.random.randn() * noise

        if i == 125:
            measuredX = 0.
        obs_U[0,0] = u
        obs_U[1,0] = measuredX

        xhat = kalmanObsd.A @ xhat + kalmanObsd.B @ obs_U

        xhatArray = np.concatenate((xhatArray, xhat), axis = 1)
        uArray = np.concatenate((uArray, u), axis = 1)
        measuredArray.append(measuredX)
        xArray = np.concatenate((xArray, np.concatenate((x, np.matrix(sigma)), axis=0)), axis = 1)

        sigma += (xhat[0,0] - xdesired[i])  * Tstep
        
        u =  -Kfeedback @ np.concatenate((xhat, np.matrix(sigma)), axis=0)
        x  = sysd.A @ x + sysd.B @ u
    
    t = np.arange(len(measuredArray))* Tstep

    plt.figure(figsize=(12,8))
    plt.plot(t,xArray[0,:].T, 'b',label = 'System')
    plt.plot(t,xhatArray[0,:].T, 'r--', label = 'xhat')
    plt.plot(t[:-1], xdesired, 'k-', alpha = 0.3, label='Desired')
    plt.plot(t,measuredArray, 'g.', label= 'Measured')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (digitizer units)')
    plt.title('Manual Simulation with Integrator')
    plt.legend()
    plt.show()

doKalmanSim(Kobs, 0)