import math


class PIDController:
    def __init__(
        self,
        Kp,
        Ki,
        Kd,
        setpoint=0.0,
        output_limits=(None, None),
        integral_limits=(None, None),
        derivative_mode="measurement",
        derivative_cutoff_hz=2.0,
    ):
        """
        PID controller with selectable derivative mode.

        derivative_mode:
            "error":
                D = Kd * d(error)/dt

                Responds to both measurement changes and setpoint changes.
                A sudden setpoint change may cause derivative kick.

            "measurement":
                D = -Kd * d(measurement)/dt

                Does not respond directly to setpoint changes.
                Usually preferred for flight-control loops.

        derivative_cutoff_hz:
            First-order low-pass-filter cutoff frequency for the
            derivative signal. Set to None or <= 0 to disable filtering.
        """

        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Kd = float(Kd)

        self.setpoint = float(setpoint)

        valid_modes = ("error", "measurement")
        if derivative_mode not in valid_modes:
            raise ValueError(
                f"derivative_mode must be one of {valid_modes}"
            )

        self.derivative_mode = derivative_mode
        self.derivative_cutoff_hz = derivative_cutoff_hz

        self._output_min, self._output_max = output_limits
        self._integral_min, self._integral_max = integral_limits

        # Internal states
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = 0.0
        self._filtered_derivative = 0.0
        self._initialized = False

        # Values available for logging
        self.error = 0.0
        self.raw_derivative = 0.0
        self.derivative = 0.0

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.unsaturated_output = 0.0
        self.output = 0.0
        self.output_saturated = False

    @staticmethod
    def _clamp(value, minimum, maximum):
        if minimum is not None:
            value = max(minimum, value)

        if maximum is not None:
            value = min(maximum, value)

        return value

    @staticmethod
    def _low_pass_alpha(cutoff_hz, dt):
        """
        First-order low-pass filter:

            filtered[k] = filtered[k-1]
                          + alpha * (raw[k] - filtered[k-1])
        """

        if cutoff_hz is None or cutoff_hz <= 0.0:
            return 1.0

        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        return dt / (tau + dt)

    def reset(self, measurement=None):
        """
        Reset the controller states.

        Passing the current measurement prevents a derivative spike on the
        first update.
        """

        self._integral = 0.0
        self._filtered_derivative = 0.0

        if measurement is None:
            self._prev_error = 0.0
            self._prev_measurement = 0.0
            self._initialized = False
        else:
            measurement = float(measurement)

            self._prev_measurement = measurement
            self._prev_error = self.setpoint - measurement
            self._initialized = True

        self.error = 0.0
        self.raw_derivative = 0.0
        self.derivative = 0.0

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.unsaturated_output = 0.0
        self.output = 0.0
        self.output_saturated = False

    def update(self, measurement, dt):
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(
                "dt must be finite, positive, and non-zero"
            )

        measurement = float(measurement)

        if not math.isfinite(measurement):
            raise ValueError("measurement must be finite")

        if not math.isfinite(self.setpoint):
            raise ValueError("setpoint must be finite")

        # ------------------------------------------------------------
        # Error and proportional term
        # ------------------------------------------------------------

        error = self.setpoint - measurement

        self.error = error
        self.P = self.Kp * error

        # ------------------------------------------------------------
        # Initialize derivative states
        # ------------------------------------------------------------

        if not self._initialized:
            self._prev_error = error
            self._prev_measurement = measurement
            self._filtered_derivative = 0.0
            self._initialized = True

        # ------------------------------------------------------------
        # Select derivative source
        # ------------------------------------------------------------

        if self.derivative_mode == "error":
            raw_derivative = (
                error - self._prev_error
            ) / dt

        else:
            # Because error = setpoint - measurement:
            #
            # d(error)/dt = -d(measurement)/dt
            #
            # when the setpoint is constant.
            raw_derivative = -(
                measurement - self._prev_measurement
            ) / dt

        self.raw_derivative = raw_derivative

        # ------------------------------------------------------------
        # Derivative low-pass filter
        # ------------------------------------------------------------

        alpha = self._low_pass_alpha(
            self.derivative_cutoff_hz,
            dt,
        )

        self._filtered_derivative += alpha * (
            raw_derivative - self._filtered_derivative
        )

        self.derivative = self._filtered_derivative
        self.D = self.Kd * self.derivative

        # Save states for next update
        self._prev_error = error
        self._prev_measurement = measurement

        # ------------------------------------------------------------
        # Integral candidate
        # ------------------------------------------------------------

        integral_candidate = self._integral + error * dt

        integral_candidate = self._clamp(
            integral_candidate,
            self._integral_min,
            self._integral_max,
        )

        candidate_I = self.Ki * integral_candidate

        candidate_output = self.P + candidate_I + self.D

        # ------------------------------------------------------------
        # Conditional integration anti-windup
        # ------------------------------------------------------------

        allow_integration = True

        if (
            self._output_max is not None
            and candidate_output > self._output_max
            and error > 0.0
        ):
            allow_integration = False

        if (
            self._output_min is not None
            and candidate_output < self._output_min
            and error < 0.0
        ):
            allow_integration = False

        if allow_integration:
            self._integral = integral_candidate

        self.I = self.Ki * self._integral

        # ------------------------------------------------------------
        # Final output
        # ------------------------------------------------------------

        self.unsaturated_output = self.P + self.I + self.D

        self.output = self._clamp(
            self.unsaturated_output,
            self._output_min,
            self._output_max,
        )

        self.output_saturated = not math.isclose(
            self.unsaturated_output,
            self.output,
            rel_tol=0.0,
            abs_tol=1e-12,
        )

        return self.output