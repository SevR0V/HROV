class Lights:
    def __init__(self, pi, pins):
        self.pi = pi
        self.pins = pins
        for pin in pins:
            self.pi.set_PWM_dutycycle(pin,0)

    def on(self):
        for pin in self.pins:
            self.pi.set_PWM_dutycycle(pin,240)
    
    def off(self):
        for pin in self.pins:
            self.pi.set_PWM_dutycycle(pin,0)

