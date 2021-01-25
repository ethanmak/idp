class PIDController:
    def __init__(self, kP, kI, kD, target, threshold):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.P = 0
        self.I = 0
        self.D = 0
        self.target = target
        self.threshold = threshold

    def update(self, value):
        error = self.target - value
        self.D = (error * self.kP - self.P) * self.kD
        self.P = error * self.kP
        if error <= self.threshold:
            self.I = 0
        else:
            self.I += self.P

        return self.P + self.I * self.kI + self.D

    def reset(self):
        self.P = 0
        self.I = 0
        self.D = 0