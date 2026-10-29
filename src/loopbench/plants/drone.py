"""Quadcopter altitude stabilization plant (Open-loop unstable double-integrator)."""

from __future__ import annotations

from typing import List
from loopbench.base import ControlTarget, GainBounds, PlantSpecs, SimulationResult
from loopbench.metrics import calculate_step_metrics
from loopbench.plot import render_ascii_plot


class DroneAltitudePlant(ControlTarget):
    """Drone Altitude 1-DoF dynamic model with gravity and quadratic aerodynamic drag.

    Equation:
      m * d^2(z)/dt^2 = Thrust - m*g - c_drag * v * |v|
    """

    def __init__(
        self,
        mass: float = 1.0,         # kg
        gravity: float = 9.81,     # m/s^2
        drag_coeff: float = 0.15,  # N/(m/s)^2
        max_thrust: float = 20.0   # Newtons (~2g thrust-to-weight ratio)
    ):
        self.m = mass
        self.g = gravity
        self.c_drag = drag_coeff
        self.u_min = 0.0           # 0 N thrust
        self.u_max = max_thrust    # 20 N thrust
        self.hover_thrust = mass * gravity  # 9.81 N required to balance gravity

    def get_specs(self) -> PlantSpecs:
        return PlantSpecs(
            plant_id="drone",
            name="Drone Vertical Altitude",
            description="Double-integrator system regulating quadcopter elevation (meters). Open-loop unstable, strictly requires damping (Kd).",
            process_variable_unit="meters",
            control_effort_unit="Newtons",
            gain_bounds=GainBounds(
                kp_min=0.0, kp_max=40.0,
                ki_min=0.0, ki_max=15.0,
                kd_min=0.0, kd_max=20.0,
            ),
            default_setpoint=5.0,
            default_duration=6.0,
            baseline_hints=(
                "Altitude is a double integrator with constant gravity downward. "
                "CRITICAL: Pure Kp will cause undamped perpetual oscillation. "
                "Derivative gain (Kd) is mandatory for stability. Integral (Ki) removes gravity sag. "
                "Recommended starting baseline: Kp=8.0, Ki=3.0, Kd=6.0."
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
        sp = setpoint if setpoint is not None else 5.0
        dur = duration if duration is not None else 6.0

        dt = 0.002  # 500 Hz
        steps = int(dur / dt)

        z = 0.0      # Altitude (m)
        v = 0.0      # Velocity (m/s)
        integral = 0.0
        prev_error = sp - z
        filtered_derivative = 0.0
        alpha_d = 0.1

        time_hist: List[float] = []
        out_hist: List[float] = []
        u_hist: List[float] = []

        disturbance_time = dur * 0.6  # Downward wind gust disturbance

        for step_idx in range(steps):
            t = step_idx * dt
            error = sp - z

            raw_d = (error - prev_error) / dt
            filtered_derivative += alpha_d * (raw_d - filtered_derivative)
            prev_error = error

            # Base PID + feedforward hover thrust assistance is what PID calculates
            u_raw = (kp * error) + (ki * integral) + (kd * filtered_derivative)
            u = max(self.u_min, min(self.u_max, u_raw))

            # Anti-windup
            if not ((u_raw > self.u_max and error > 0) or (u_raw < self.u_min and error < 0)):
                integral += error * dt

            # Wind gust disturbance (downward force)
            wind_force = disturbance_step if t >= disturbance_time else 0.0

            time_hist.append(round(t, 4))
            out_hist.append(round(z, 4))
            u_hist.append(round(u, 4))

            # RK4 Integration for altitude and velocity
            def derivatives(curr_z: float, curr_v: float, thrust: float):
                drag = self.c_drag * curr_v * abs(curr_v)
                accel = (thrust - (self.m * self.g) - drag - wind_force) / self.m
                # Ground collision constraint: cannot fall below z=0
                if curr_z <= 0.0 and accel < 0:
                    accel = 0.0
                return curr_v, accel

            k1_z, k1_v = derivatives(z, v, u)
            k2_z, k2_v = derivatives(z + 0.5 * dt * k1_z, v + 0.5 * dt * k1_v, u)
            k3_z, k3_v = derivatives(z + 0.5 * dt * k2_z, v + 0.5 * dt * k2_v, u)
            k4_z, k4_v = derivatives(z + dt * k3_z, v + dt * k3_v, u)

            z += (dt / 6.0) * (k1_z + 2 * k2_z + 2 * k3_z + k4_z)
            v += (dt / 6.0) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)

            if z < 0.0:
                z = 0.0
                v = 0.0

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
            plant_id="drone",
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
