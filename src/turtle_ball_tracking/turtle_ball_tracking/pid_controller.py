class PIDController:
    def __init__(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._prev_error = 0.0
        self._integral = 0.0

    def compute(self, error):
        p = self._kp * error
        self._integral += error
        i = self._ki * self._integral
        d = self._kd * (error - self._prev_error)
        self._prev_error = error
        return p + i + d
