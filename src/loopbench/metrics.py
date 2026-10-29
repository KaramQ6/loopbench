"""Deterministic control metrics calculation (step_info, ITAE, stability)."""

from __future__ import annotations

import math
from typing import List
from pydantic import BaseModel


class StepMetrics(BaseModel):
    """Standard control engineering response metrics."""
    rise_time: float | None = None       # Time from 10% to 90% of setpoint
    settling_time: float | None = None   # Time to remain within +/- 2% error band
    overshoot_pct: float = 0.0           # Peak overshoot percentage: max(0, (peak - sp) / sp * 100)
    steady_state_error: float = 0.0      # abs(final_value - setpoint)
    itae_score: float = 0.0              # Integral of Time-weighted Absolute Error (lower is better)
    saturation_pct: float = 0.0          # Percentage of time control effort spent at actuator limits
    is_stable: bool = True               # True if no divergence / non-finite values occurred
    performance_score: float = 0.0       # Normalized benchmark score (0 - 100)


def calculate_step_metrics(
    time_series: List[float],
    output_series: List[float],
    control_effort_series: List[float],
    setpoint: float,
    u_min: float,
    u_max: float,
    tolerance_band: float = 0.02,
) -> StepMetrics:
    """Calculate standard step response characteristics deterministically.

    Why deterministic: LLMs should exercise strategic control choices, while
    mathematical metrics must remain rigorous, reproducible, and verifiable.
    """
    if not time_series or not output_series or len(time_series) != len(output_series):
        return StepMetrics(is_stable=False, performance_score=0.0)

    n = len(time_series)
    final_output = output_series[-1]

    # 1. Stability check: Detect NaN, Inf, or extreme divergence
    for val in output_series:
        if math.isnan(val) or math.isinf(val) or abs(val) > 1e6:
            return StepMetrics(
                is_stable=False,
                steady_state_error=float("inf"),
                itae_score=1e6,
                performance_score=0.0,
            )

    # 2. Overshoot calculation
    peak_val = max(output_series)
    if setpoint > 0:
        overshoot_pct = max(0.0, ((peak_val - setpoint) / setpoint) * 100.0)
    elif setpoint < 0:
        min_val = min(output_series)
        overshoot_pct = max(0.0, ((setpoint - min_val) / abs(setpoint)) * 100.0)
    else:
        overshoot_pct = max(0.0, abs(peak_val) * 100.0)

    # 3. Steady-state error
    steady_state_error = abs(final_output - setpoint)

    # 4. Rise time (10% to 90% of setpoint)
    t_10: float | None = None
    t_90: float | None = None
    target_10 = 0.10 * setpoint
    target_90 = 0.90 * setpoint

    for t, y in zip(time_series, output_series):
        if t_10 is None and y >= target_10:
            t_10 = t
        if t_90 is None and y >= target_90:
            t_90 = t
            break

    rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None and t_90 >= t_10) else None

    # 5. Settling time (stays within 2% band of setpoint until the end)
    band = tolerance_band * abs(setpoint) if setpoint != 0 else tolerance_band
    settling_time: float | None = None
    for i in range(n - 1, -1, -1):
        if abs(output_series[i] - setpoint) > band:
            if i + 1 < n:
                settling_time = time_series[i + 1]
            else:
                settling_time = None
            break
    else:
        # If it was within the band the entire time
        settling_time = time_series[0]

    # 6. ITAE (Integral of Time-weighted Absolute Error) & Saturation
    itae = 0.0
    saturated_count = 0

    for i in range(1, n):
        dt = time_series[i] - time_series[i - 1]
        t = time_series[i]
        error = abs(setpoint - output_series[i])
        itae += t * error * dt

        u = control_effort_series[i]
        if u <= u_min + 1e-4 or u >= u_max - 1e-4:
            saturated_count += 1

    saturation_pct = (saturated_count / max(1, n)) * 100.0

    # 7. Normalized benchmark score (0 - 100 scale)
    # Penalizes ITAE, extreme overshoot (>30%), and sluggish settling
    score = 100.0 / (1.0 + 0.15 * itae + 0.5 * (overshoot_pct / 10.0) + 2.0 * steady_state_error)
    score = max(0.0, min(100.0, score))

    return StepMetrics(
        rise_time=round(rise_time, 4) if rise_time is not None else None,
        settling_time=round(settling_time, 4) if settling_time is not None else None,
        overshoot_pct=round(overshoot_pct, 2),
        steady_state_error=round(steady_state_error, 4),
        itae_score=round(itae, 3),
        saturation_pct=round(saturation_pct, 2),
        is_stable=True,
        performance_score=round(score, 2),
    )
