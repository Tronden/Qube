import sys


class PID:
    def __init__(self):
        print("Initializing PID...")

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.windup_guard = 0.0
        """""
        self.kp = 0.003
        self.ki = 0.005
        self.kd = 0.0003
        self.windup_guard = 1000.0

        
        self.kp = 0.008
        self.ki = 0.0025
        self.kd = 0.0001
        self.windup_guard = 100000.0
        """""
        self.lastIntegral = 0
        self.lastError = 0
        self.pkp = 0.0
        self.pki = 0.0
        self.pkd = 0.0
        self.useWindup = False

    def regulate(self, current_value, delta_time):
        error = self.setpoint - current_value

        if delta_time <= 0:
            return 0

        delta_error = error - self.last_error
        self.integral += error * delta_time
        derivative = delta_error / delta_time if delta_time > 0 else 0

        if self.integral > self.windup_guard > 0:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard < 0:
            self.integral = -self.windup_guard

        self.pkp = self.kp * error
        self.pki = self.ki * self.integral
        self.pkd = self.kd * derivative
        
        output = self.pkp + self.pki + self.pkd

        # Printing the values for debugging purposes
        print(error, delta_time, self.pkp, self.pki, self.pkd, f"output={output}")
        self.last_error = error
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0  # Reset integral
        self.last_error = 0  # Reset last error

    def copy(self, pid):
        self.kp = pid.kp
        self.ki = pid.ki
        self.kd = pid.kd
        self.windup = pid.windup_guard
        self.useWindup = pid.useWindup
