"""DC Motor velocity & position control plant (2nd-order electromechanical ODE)."""

from __future__ import annotations

from typing import List
from loopbench.base import ControlTarget, GainBounds, PlantSpecs, SimulationResult
from loopbench.metrics import calculate_step_metrics
from loopbench.plot import render_ascii_plot


class DCMotorPlant(ControlTarget):
    """Permanent-Magnet DC Motor velocity control model.

    Equations of motion:
      d(omega)/dt = (K_t * i - b * omega - load_torque) / J
      di/dt       = (V - R * i - K_e * omega) / L
    """

    def __init__(
        self,
        j_inertia: float = 0.01,    # Rotor inertia (kg*m^2)
        b_friction: float = 0.01,   # Viscous friction (N*m*s)
        k_torque: float = 0.05,     # Motor torque constant (N*m/A)
        k_emf: float = 0.05,        # Back-EMF constant (V/(rad/s))
        r_armature: float = 1.0,    # Armature resistance (Ohms)
        l_inductance: float = 0.005 # Armature inductance (H)
    ):
        self.j = j_inertia
        self.b = b_friction
        self.kt = k_torque
        self.ke = k_emf
        self.r = r_armature
        self.l = l_inductance
        self.v_min = -12.0
        self.v_max = 12.0

    def get_specs(self) -> PlantSpecs:
        return PlantSpecs(
            plant_id="dc_motor",
            name="DC Motor Angular Velocity",
            description="2nd-order electromechanical motor regulating shaft speed (rad/s). Fast dynamics.",
            process_variable_unit="rad/s",
            control_effort_unit="Volts",
            gain_bounds=GainBounds(
                kp_min=0.0, kp_max=50.0,
                ki_min=0.0, ki_max=100.0,
                kd_min=0.0, kd_max=5.0,
            ),
            default_setpoint=10.0,
            default_duration=3.0,
            baseline_hints=(
                "DC motor velocity loop: Moderate inertia and low inductance. "
                "Recommended starting baseline: Kp=2.5, Ki=5.0, Kd=0.05. "
                "Watch out for voltage saturation (+/- 12V) during aggressive step starts."
            )
        )

    def step(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float | None = None,
        duration: float | None = None,
        disturbance_step: float = 0.0,
    ) -> SimulationResult:
        sp = setpoint if setpoint is not None else 10.0
        dur = duration if duration is not None else 3.0

        # Numerical integration configuration (RK4 at 1 kHz)
        dt = 0.001
        steps = int(dur / dt)

        # State variables
        omega = 0.0  # rad/s
        current = 0.0  # Amperes

        # PID controller internal state
        integral = 0.0
        prev_error = sp - omega
        filtered_derivative = 0.0
        alpha_d = 0.1  # Low-pass filter coefficient for derivative term to avoid spikes

        time_hist: List[float] = []
        out_hist: List[float] = []
        u_hist: List[float] = []

        disturbance_time = dur * 0.6  # inject disturbance at 60% of test if requested

        for step_idx in range(steps):
            t = step_idx * dt
            error = sp - omega

            # Disturbance torque (e.g. mechanical load step)
            load_torque = disturbance_step if t >= disturbance_time else 0.0

            # Filtered derivative
            raw_d = (error - prev_error) / dt
            filtered_derivative += alpha_d * (raw_d - filtered_derivative)
            prev_error = error

            # Calculate raw PID voltage
            u_raw = (kp * error) + (ki * integral) + (kd * filtered_derivative)

            # Clamp control effort (actuator saturation)
            u = max(self.v_min, min(self.v_max, u_raw))

            # Anti-windup: conditional integration (clamping)
            if not ((u_raw > self.v_max and error > 0) or (u_raw < self.v_min and error < 0)):
                integral += error * dt

            # Record history
            time_hist.append(round(t, 4))
            out_hist.append(round(omega, 4))
            u_hist.append(round(u, 4))

            # RK4 Integration for the 2 coupled 1st-order equations
            def derivatives(w: float, i: float, v: float, tl: float):
                dw = (self.kt * i - self.b * w - tl) / self.j
                di = (v - self.r * i - self.ke * w) / self.l
                return dw, di

            k1_w, k1_i = derivatives(omega, current, u, load_torque)
            k2_w, k2_i = derivatives(omega + 0.5 * dt * k1_w, current + 0.5 * dt * k1_i, u, load_torque)
            k3_w, k3_i = derivatives(omega + 0.5 * dt * k2_w, current + 0.5 * dt * k2_i, u, load_torque)
            k4_w, k4_i = derivatives(omega + dt * k3_w, current + dt * k3_i, u, load_torque)

            omega += (dt / 6.0) * (k1_w + 2 * k2_w + 2 * k3_w + k4_w)
            current += (dt / 6.0) * (k1_i + 2 * k2_i + 2 * k3_i + k4_i)

        metrics = calculate_step_metrics(
            time_series=time_hist,
            output_series=out_hist,
            control_effort_series=u_hist,
            setpoint=sp,
            u_min=self.v_min,
            u_max=self.v_max,
        )

        plot = render_ascii_plot(time_hist, out_hist, sp)

        return SimulationResult(
            plant_id="dc_motor",
            kp=kp,
            ki=ki,
            kd=kd,
            setpoint=sp,
            duration=dur,
            metrics=metrics,
            plot_ascii=plot,
            time=time_hist,
            output=out_hist,
            control_effort=u_hist,
        )
