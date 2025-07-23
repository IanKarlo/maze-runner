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

    def init_encoders(self, cbf = None):
        if cbf:
            gpio.setup_interruptions(self.ENC1, cbf)
            gpio.setup_interruptions(self.ENC2, cbf)    
        else:
            gpio.setup_interruptions(self.ENC1)
            gpio.setup_interruptions(self.ENC2)    
    
    
    def run(self, duty_cycle = 50, direction='forward'):
        if direction == 'forward':
            gpio.digital_write(self.IN1, 0)
            gpio.digital_write(self.IN2, 1)
        else:
            gpio.digital_write(self.IN1, 1)
            gpio.digital_write(self.IN2, 0)

        gpio.set_PWM(self.ENA, duty_cycle)

    

class MotorDriver():
    def __init__(self, cbf = None, stop_fn_a = None, stop_fn_b = None):
        gpio.init()
        self.motor1 = Motor(12, 27, 22, 20, 21)
        self.motor2 = Motor(13, 23, 24, 5, 6)
        #motor1
        self.motor1.init_h_bridge()
        self.motor1.init_encoders(cbf)
        #motor2
        self.motor2.init_h_bridge()
        self.motor2.init_encoders(cbf)

        self.stop_fn_a = stop_fn_a
        self.stop_fn_b = stop_fn_b

    def run_motors(self, direction1, duty_cycle1, direction2, duty_cycle2):
        self.motor1.run(duty_cycle1, direction1)
        self.motor2.run(duty_cycle2, direction2)

        changed_a = False
        changed_b = False

        while True:
            if self.stop_fn_a() and not changed_a:
                self.motor1.run(0)
                changed_a = True
            if self.stop_fn_b() and not changed_b:
                self.motor2.run(0)
                changed_b = True
            if self.stop_fn_a() and self.stop_fn_b():
                print("hey")
                break