import numpy as np


def normalized(vector):
    vector = np.array(vector)
    result = np.zeros_like(vector)
    length = np.sqrt(np.sum(vector**2))
    if length > 0.001:
        result = vector / length
    return result


def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def constrain(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def super_map(x, in_min, in_max, out_min, out_max, out0_dz, out_bounds_dz, in_zero_zone):
    x0 = (in_max+in_min)/2
    y0 = map_value(x0, in_min, in_max, out_min, out_max)
    if x < x0 - in_zero_zone:
        value = map_value(x, in_min, x0, out_min + out_bounds_dz[0], y0-out0_dz[0])
    elif x > x0 + in_zero_zone:
        value = map_value(x, x0, in_max, y0 + out0_dz[1], out_max-out_bounds_dz[1])
    else:
        value = y0
    return value

class ExpMovingAverageFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None
        
    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
        #return new_value

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, isAngles = False):
        self.isAngles = isAngles
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def update(self, measured_value, dt):
        error = None
        if self.isAngles:
            error = (self.setpoint - measured_value + 180) % 360 - 180
        else:
            error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

    def setConstants(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
            self.integral = 0
        if Kd is not None:
            self.Kd = Kd
            self.last_error = 0

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0
