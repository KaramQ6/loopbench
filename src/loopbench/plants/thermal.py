"""Thermal chamber heating plant (First-Order Plus Dead-Time: FOPDT)."""

from __future__ import annotations

from collections import deque
from typing import List
from loopbench.base import ControlTarget, GainBounds, PlantSpecs, SimulationResult
from loopbench.metrics import calculate_step_metrics
from loopbench.plot import render_ascii_plot


class ThermalPlant(ControlTarget):
    """First-Order Plus Dead-Time (FOPDT) Thermal chamber system.

    Differential equation:
      tau * dT/dt = -(T - T_ambient) + K_gain * u(t - dead_time)
    """

    def __init__(
        self,
        tau: float = 5.0,           # Thermal time constant in seconds
        k_gain: float = 0.8,        # Process gain (deg C per % power)
        dead_time: float = 0.8,     # Sensor transport/thermal lag in seconds
        ambient_temp: float = 25.0  # Ambient room temperature in deg C
    ):
        self.tau = tau
        self.k = k_gain
        self.theta = dead_time
        self.ambient = ambient_temp
        self.u_min = 0.0            # Heater power 0%
        self.u_max = 100.0          # Heater power 100%

    def get_specs(self) -> PlantSpecs:
        return PlantSpecs(
            plant_id="thermal",
            name="Thermal Oven Chamber",
            description="First-Order Plus Dead-Time (FOPDT) process regulating oven temperature (deg C). Sluggish with transport delay.",
            process_variable_unit="°C",
            control_effort_unit="% Power",
            gain_bounds=GainBounds(
                kp_min=0.0, kp_max=20.0,
                ki_min=0.0, ki_max=5.0,
                kd_min=0.0, kd_max=10.0,
            ),
            default_setpoint=60.0,
            default_duration=25.0,
            baseline_hints=(
                "Thermal processes have significant dead time (0.8s) and inertia (tau=5.0s). "
                "Aggressive Ki or Kd will induce severe sustained oscillations. "
                "Recommended starting baseline: Kp=2.0, Ki=0.3, Kd=0.5."
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
        sp = setpoint if setpoint is not None else 60.0
        dur = duration if duration is not None else 25.0

        dt = 0.02  # 50 Hz simulation is plenty for slow thermal dynamics
        steps = int(dur / dt)
        delay_steps = max(1, int(self.theta / dt))

        # Dead-time delay buffer
        u_buffer: deque[float] = deque([0.0] * delay_steps, maxlen=delay_steps)

        temp = self.ambient
        integral = 0.0
        prev_error = sp - temp
        filtered_derivative = 0.0
        alpha_d = 0.05

        time_hist: List[float] = []
        out_hist: List[float] = []
        u_hist: List[float] = []

        disturbance_time = dur * 0.65

        for step_idx in range(steps):
            t = step_idx * dt
            error = sp - temp

            raw_d = (error - prev_error) / dt
            filtered_derivative += alpha_d * (raw_d - filtered_derivative)
            prev_error = error

            u_raw = (kp * error) + (ki * integral) + (kd * filtered_derivative)
            u = max(self.u_min, min(self.u_max, u_raw))

            # Anti-windup
            if not ((u_raw > self.u_max and error > 0) or (u_raw < self.u_min and error < 0)):
                integral += error * dt

            # Delayed actuation
            u_delayed = u_buffer[0]
            u_buffer.append(u)

            # Environmental disturbance (e.g. oven door opened, cooling loss)
            cooling_loss = disturbance_step if t >= disturbance_time else 0.0

            time_hist.append(round(t, 4))
            out_hist.append(round(temp, 4))
            u_hist.append(round(u, 4))

            # Euler integration for 1st order lag
            dt_temp = (-(temp - self.ambient) + (self.k * u_delayed) - cooling_loss) / self.tau
            temp += dt_temp * dt

        metrics = calculate_step_metrics(
            time_series=time_hist,
            output_series=out_hist,
            control_effort_series=u_hist,
            setpoint=sp,
            u_min=self.u_min,
            u_max=self.u_max,
        )

        plot = render_ascii_plot(time_hist, out_hist, sp)

        return SimulationResult(
            plant_id="thermal",
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
