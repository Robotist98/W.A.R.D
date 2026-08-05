class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = None

    def reset(self):
        """Clear accumulated state when an axis is intentionally held still."""
        self.integral = 0
        self.previous_error = None

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (
            (error - self.previous_error) / dt
            if self.previous_error is not None and dt > 0
            else 0
        )

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error

        return output
