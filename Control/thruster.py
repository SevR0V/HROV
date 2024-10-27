import time
from utils import map_value, constrain, super_map


class Thrusters:
    def __init__(self, pi, pins, en_pins, pulse_zero_deadzones=None, pulse_bounds_deadzones=None):
        self.pi = pi
        self.thrust_values = [0] * len(pins)
        self.pins = pins
        self.en_pins = en_pins
        self.pulse_zero_deadzones = pulse_zero_deadzones or [0] * len(pins)
        self.pulse_bounds_deadzones = pulse_bounds_deadzones or [0] * len(pins)
        self.min_max_pulse = (1000, 2000)
        self.zero_control_zone = 0.1
        for pin in self.en_pins:
            self.pi.write(pin, 0)

    def initialize(self):
        for pin in self.en_pins:
            self.pi.write(pin,0)
            time.sleep(0.5)
        for pin in self.pins:
            self.pi.set_servo_pulsewidth(pin,1500)
        for pin in self.en_pins:
            self.pi.write(pin,1)
            time.sleep(0.5)
        time.sleep(2)
        print('Thrusters init complete')

    def get_thrust(self):
        return self.thrust_values

    def set_thrust_all(self, values):
        min_pulse, max_pulse = self.min_max_pulse
        for pin, thrust, y0_dz, y1_dz in zip(self.pins, self.thrust_values, self.pulse_zero_deadzones, self.pulse_bounds_deadzones):
            thrust = constrain(thrust,-100, 100)
            pulse_width = super_map(thrust, -100, 100, min_pulse, max_pulse, y0_dz, y1_dz, self.zero_control_zone)
            self.pi.set_servo_pulsewidth(pin, pulse_width)

    def set_thrust(self, idx, value):
        pin = self.pins[idx]
        min_pulse, max_pulse = self.min_max_pulse
        thrust = constrain(value, -100, 100) * self.direction_mask[idx]
        pulse_width = map_value(thrust, -100, 100, min_pulse, max_pulse)
        print(pulse_width)
        self.pi.set_servo_pulsewidth(pin, pulse_width)

    def off(self):
        for pin in self.en_pins:
            self.pi.write(pin,0)
        for pin in self.pins:
            self.pi.set_servo_pulsewidth(pin, 0)



