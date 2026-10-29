"""Tests for deterministic control metric calculations."""

import math
from loopbench.metrics import calculate_step_metrics


def test_ideal_step_response():
    """Verify metrics on an ideal first-order exponential response."""
    time = [i * 0.01 for i in range(200)]  # 0 to 2.0s
    sp = 10.0
    # y(t) = 10 * (1 - e^(-5t))
    output = [sp * (1.0 - math.exp(-5.0 * t)) for t in time]
    u = [1.0 for _ in time]

    metrics = calculate_step_metrics(
        time_series=time,
        output_series=output,
        control_effort_series=u,
        setpoint=sp,
        u_min=-10.0,
        u_max=10.0,
    )

    assert metrics.is_stable is True
    assert metrics.overshoot_pct == 0.0
    assert metrics.steady_state_error < 0.01
    assert metrics.rise_time is not None
    assert 0.3 < metrics.rise_time < 0.6  # ln(9)/5 ~= 0.44s
    assert metrics.settling_time is not None
    assert metrics.settling_time < 1.0
    assert metrics.performance_score > 75.0


def test_overshoot_detection():
    """Verify overshoot percentage is computed accurately on peak values."""
    time = [0.0, 0.5, 1.0, 1.5, 2.0]
    output = [0.0, 5.0, 13.0, 10.2, 10.0]  # Setpoint 10.0, peak 13.0 => 30% overshoot
    u = [1.0] * 5

    metrics = calculate_step_metrics(
        time_series=time,
        output_series=output,
        control_effort_series=u,
        setpoint=10.0,
        u_min=-10.0,
        u_max=10.0,
    )

    assert metrics.is_stable is True
    assert metrics.overshoot_pct == 30.0
    assert metrics.steady_state_error == 0.0


def test_unstable_divergence():
    """Verify that divergent, NaN, or infinite signals are flagged as unstable."""
    time = [0.0, 1.0, 2.0]
    output = [0.0, 10.0, float("nan")]
    u = [0.0, 0.0, 0.0]

    metrics = calculate_step_metrics(
        time_series=time,
        output_series=output,
        control_effort_series=u,
        setpoint=10.0,
        u_min=-10.0,
        u_max=10.0,
    )

    assert metrics.is_stable is False
    assert metrics.performance_score == 0.0
