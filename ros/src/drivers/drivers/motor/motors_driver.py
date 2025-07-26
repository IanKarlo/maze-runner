from . import gpio

class Motor():
    def __init__(self, ENA, IN1, IN2, ENC1, ENC2):
        self.ENA = ENA
        self.IN1 = IN1
        self.IN2 = IN2
        self.ENC1 = ENC1
        self.ENC2 = ENC2

    def init_h_bridge(self):
        gpio.pin_mode(self.ENA, 'out')
        gpio.pin_mode(self.IN1, 'out')
        gpio.pin_mode(self.IN2, 'out')
        gpio.digital_write(self.IN1, 0)
        gpio.digital_write(self.IN2, 0)

    def init_encoders(self, cbf = None):
        if cbf:
            gpio.setup_interruptions(self.ENC1, cbf)
            gpio.setup_interruptions(self.ENC2, cbf)    
        else:
            gpio.setup_interruptions(self.ENC1)
            gpio.setup_interruptions(self.ENC2)    
    
    
    def run(self, duty_cycle = 50):
        if duty_cycle > 0:
            gpio.digital_write(self.IN1, 0)
            gpio.digital_write(self.IN2, 1)
        else:
            gpio.digital_write(self.IN1, 1)
            gpio.digital_write(self.IN2, 0)

        gpio.set_PWM(self.ENA, duty_cycle)

    

class MotorDriver():
    def __init__(self, cbf = None):
        gpio.init()
        self.motor1 = Motor(12, 27, 22, 20, 21)
        self.motor2 = Motor(13, 23, 24, 5, 6)
        #motor1
        self.motor1.init_h_bridge()
        self.motor1.init_encoders(cbf)
        #motor2
        self.motor2.init_h_bridge()
        self.motor2.init_encoders(cbf)

    def run_motors(self, duty_cycle1, duty_cycle2):
        self.motor1.run(duty_cycle1)
        self.motor2.run(duty_cycle2)
