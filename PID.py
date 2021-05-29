class PID:
    Kp = 1.0
    Ki = 1.0
    Kd = 1.0
    limMin = 0.0
    limMax = 0.0

    integral = 0.0
    lastError = 0.0
    lastMeasurement = 0.0

    def __init__(self, Kp, Ki, Kd, limMin, limMax):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limMin = limMin
        self.limMax = limMax

    def update(self, target, actual):
        error = target - actual

        proportional = self.Kp * error
        self.integral += self.Ki * ((error + self.lastError) / 2.0)
        derivative = self.Kd * (actual - self.lastMeasurement)

        limMinInt = 0.0
        limMaxInt = 0.0
        if proportional < self.limMax:
            limMaxInt = self.limMax
        else:
            limMaxInt = 0.0
        if proportional > self.limMin:
            limMinInt = self.limMin
        else:
            limMinInt = 0.0

        self.integral = self.clamp(self.integral, limMinInt, limMaxInt)

        self.lastError = error
        self.lastMeasurement = actual

        return proportional + self.integral + derivative

    def reset(self):
        self.lastError = 0
        self.lastMeasurement = 0
        self.integral = 0

    def clamp(self, num, min_value, max_value):
        return max(min(num, max_value), min_value)
