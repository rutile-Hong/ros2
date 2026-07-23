
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(None, None), integral_limits=(None, None), derivative_on_measurement = True):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = setpoint

        # Internal state
        self._prev_error = 0.0
        self._integral = 0.0

        self._prev_measurement = 0.0
        self.derivative_on_measurement = derivative_on_measurement

        # Output and integral clamping
        self._output_min, self._output_max = output_limits
        self._integral_min, self._integral_max = integral_limits

    def reset(self):
        self._prev_error = 0.0
        self._prev_measurement = 0.0
        self._integral = 0.0

    def update(self, measurement, dt):
        if dt <= 0.0:
            raise ValueError("dt must be positive and non-zero")

        # Error computation
        error = self.setpoint - measurement

        # Proportional term
        P = self.Kp * error

        # Integral term
        self._integral += error * dt
        if self._integral_min is not None:
            self._integral = max(self._integral_min, self._integral)
        if self._integral_max is not None:
            self._integral = min(self._integral_max, self._integral)
        I = self.Ki * self._integral

        # Derivative term
        if self.derivative_on_measurement:
            # Derivative on measurement
            derivative = -(measurement - self._prev_measurement) / dt
            self._prev_measurement = measurement
        else:
            # Derivative on error
            derivative = (error - self._prev_error) / dt
            self._prev_error = error

        D = self.Kd * derivative

        # Final output
        output = P + I + D

        # Apply output limits
        if self._output_min is not None:
            output = max(self._output_min, output)
        if self._output_max is not None:
            output = min(self._output_max, output)

        return output