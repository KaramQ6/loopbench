"""Tests for simulated dynamic plants and FastMCP tools."""

from loopbench.plants import get_plant, list_available_plants
from loopbench.server import list_plants, get_tuning_hints, step_response, benchmark_run


def test_list_plants():
    plants = list_available_plants()
    ids = [p.plant_id for p in plants]
    assert "dc_motor" in ids
    assert "thermal" in ids
    assert "drone" in ids


def test_dc_motor_baseline_simulation():
    motor = get_plant("dc_motor")
    res = motor.step(kp=2.5, ki=5.0, kd=0.05, setpoint=10.0, duration=2.0)

    assert res.plant_id == "dc_motor"
    assert res.metrics.is_stable is True
    assert res.metrics.steady_state_error < 0.2  # Ki brings steady state error near zero
    assert len(res.plot_ascii) > 50
    assert "*" in res.plot_ascii


def test_thermal_fopdt_delay():
    thermal = get_plant("thermal")
    res = thermal.step(kp=2.0, ki=0.3, kd=0.5, setpoint=60.0, duration=15.0)

    assert res.plant_id == "thermal"
    assert res.metrics.is_stable is True
    assert res.output[0] == 25.0  # Room temp
    assert res.output[-1] > 55.0  # Heats up towards 60


def test_drone_altitude_requires_kd():
    drone = get_plant("drone")

    # With good damping (Kd=6.0)
    res_stable = drone.step(kp=8.0, ki=3.0, kd=6.0, setpoint=5.0, duration=4.0)
    assert res_stable.metrics.is_stable is True
    assert res_stable.metrics.overshoot_pct < 40.0

    # With zero damping (Kd=0.0), oscillations should be much worse
    res_undamped = drone.step(kp=8.0, ki=0.0, kd=0.0, setpoint=5.0, duration=4.0)
    assert res_undamped.metrics.overshoot_pct > res_stable.metrics.overshoot_pct


def test_fastmcp_tools():
    # 1. list_plants tool
    all_plants = list_plants()
    assert len(all_plants) == 3

    # 2. get_tuning_hints tool
    hints = get_tuning_hints("dc_motor")
    assert "gain_bounds" in hints
    assert hints["plant_id"] == "dc_motor"

    # 3. step_response tool with valid gains
    resp = step_response(plant_id="dc_motor", kp=2.0, ki=4.0, kd=0.05)
    assert "metrics" in resp
    assert resp["metrics"]["is_stable"] is True

    # 4. step_response tool bounds validation
    err_resp = step_response(plant_id="dc_motor", kp=999.0, ki=1.0, kd=0.0)
    assert "error" in err_resp

    # 5. benchmark_run tool
    bench = benchmark_run(plant_id="dc_motor", kp=2.5, ki=5.0, kd=0.05)
    assert "benchmark_score" in bench
    assert bench["benchmark_score"] > 0.0
