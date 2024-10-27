import pigpio
from utils import map_value, constrain

class Servo:
    def __init__(self, pi: pigpio, pin, valuesLimits = [0, 90]):
        self.pi = pi
        self.pin = pin
        self.limits = valuesLimits

    def rotate(self, angle):
        angle = constrain(angle, self.limits[0], self.limits[1])
        pulse_width = map_value(angle, self.limits[0], self.limits[1], 1000, 2000)
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)
