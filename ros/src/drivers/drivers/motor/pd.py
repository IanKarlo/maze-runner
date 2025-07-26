class PDController:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.e_prev = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        derivative = (error - self.e_prev)
        output = self.Kp * error + self.Kd * derivative
        self.e_prev = error
        return output