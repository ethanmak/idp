class PIDController:
    """
    A basic PID Controller with individual kP, kI, and kD coefficients
    """
    def __init__(self, kP, kI, kD, target, threshold):
        """
        Initializer
        :param kP: Constant multiplied by error
        :param kI: Constant multiplied by integral
        :param kD: Constant multiplied by derivative
        :param target: Target value of controller
        :param threshold: Threshold under which I term is neglected
        """
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.P = 0
        self.I = 0
        self.D = 0
        self.target = target
        self.threshold = threshold

    def update(self, value: float):
        """
        Updates controller with sensor value and returns output

        :param value: Input value of sensor or controller
        :return: Output of PID controller
        """
        error = self.target - value
        self.D = (error * self.kP - self.P) * self.kD
        self.P = error * self.kP
        if abs(error) <= self.threshold:  # reset I if error within threshold to prevent exploding I terms
            self.I = 0
        else:
            self.I += self.P
        return self.P + self.I * self.kI + self.D

    def reset(self):
        """
        Reset updated values of P, I, and D

        :return: None
        """
        self.P = 0
        self.I = 0
        self.D = 0
