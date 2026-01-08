class SimplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.previous_error = 0.0
    
    def compute(self, error):

        p = self.kp * error

        self.integral += error

        i = self.ki * self.integral

        d = self.kd * (error - self.previous_error)

        self.previous_error = error

        return p + i + d
    
pid = SimplePID(2.0, 0.1, 0.05)
error = 5.0
output = pid.compute(error)

print(f"Control Output: {output}")