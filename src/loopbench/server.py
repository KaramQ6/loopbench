"""FastMCP server entrypoint for LoopBench."""

from __future__ import annotations

import sys
from typing import Any, Dict, List
from mcp.server.fastmcp import FastMCP

from loopbench.plants import get_plant, list_available_plants

# Initialize FastMCP Server
mcp = FastMCP("loopbench")


@mcp.tool()
def list_plants() -> List[Dict[str, Any]]:
    """List all available physical plant simulation targets and their specifications.

    Returns:
        List of plant specs including plant_id, name, description, physical units,
        recommended PID gain bounds, and default setpoint.
    """
    specs = list_available_plants()
    return [s.model_dump() for s in specs]


@mcp.tool()
def get_tuning_hints(plant_id: str) -> Dict[str, Any]:
    """Retrieve classical control heuristics and safety bounds for a target plant.

    Args:
        plant_id: Identifier of the plant ('dc_motor', 'thermal', or 'drone').

    Returns:
        Baseline gain suggestions, dynamic properties, and safe tuning boundaries.
    """
    plant = get_plant(plant_id)
    specs = plant.get_specs()
    return {
        "plant_id": specs.plant_id,
        "name": specs.name,
        "gain_bounds": specs.gain_bounds.model_dump(),
        "baseline_hints": specs.baseline_hints,
        "default_setpoint": specs.default_setpoint,
        "units": {
            "process_variable": specs.process_variable_unit,
            "control_effort": specs.control_effort_unit,
        },
    }


@mcp.tool()
def step_response(
    plant_id: str,
    kp: float,
    ki: float,
    kd: float,
    setpoint: float | None = None,
    duration: float | None = None,
) -> Dict[str, Any]:
    """Simulate a closed-loop step response with specific PID gains.

    Args:
        plant_id: Identifier of the plant ('dc_motor', 'thermal', or 'drone').
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        setpoint: Target process variable value (optional, uses default if omitted).
        duration: Test duration in seconds (optional, uses default if omitted).

    Returns:
        Evaluation bundle containing:
          - metrics: rise time, settling time, overshoot %, steady-state error, ITAE, stability
          - plot_ascii: Visual markdown step response curve with setpoint line
          - performance_score: Normalized tuning quality score (0 - 100)
    """
    plant = get_plant(plant_id)
    specs = plant.get_specs()
    bounds = specs.gain_bounds

    # Safety clamp validation
    if not (bounds.kp_min <= kp <= bounds.kp_max):
        return {
            "error": f"Kp={kp} out of bounds [{bounds.kp_min}, {bounds.kp_max}] for plant '{plant_id}'"
        }
    if not (bounds.ki_min <= ki <= bounds.ki_max):
        return {
            "error": f"Ki={ki} out of bounds [{bounds.ki_min}, {bounds.ki_max}] for plant '{plant_id}'"
        }
    if not (bounds.kd_min <= kd <= bounds.kd_max):
        return {
            "error": f"Kd={kd} out of bounds [{bounds.kd_min}, {bounds.kd_max}] for plant '{plant_id}'"
        }

    res = plant.step(kp=kp, ki=ki, kd=kd, setpoint=setpoint, duration=duration)

    return {
        "plant_id": res.plant_id,
        "gains": {"kp": res.kp, "ki": res.ki, "kd": res.kd},
        "setpoint": res.setpoint,
        "duration_seconds": res.duration,
        "metrics": res.metrics.model_dump(),
        "plot_ascii": res.plot_ascii,
    }


@mcp.tool()
def benchmark_run(
    plant_id: str,
    kp: float,
    ki: float,
    kd: float,
    disturbance_step: float | None = None,
) -> Dict[str, Any]:
    """Test disturbance rejection resilience and grade control quality under load.

    Args:
        plant_id: Identifier of the plant ('dc_motor', 'thermal', or 'drone').
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        disturbance_step: Magnitude of external disturbance step (optional default applied).

    Returns:
        Benchmark score and disturbance recovery evaluation under sudden load.
    """
    plant = get_plant(plant_id)

    # Sensible default disturbance magnitudes per plant
    default_disturbances = {
        "dc_motor": 0.5,    # 0.5 N*m load torque
        "thermal": 15.0,    # 15.0 °C ambient cooling blast
        "drone": 3.0,       # 3.0 N wind gust downward
    }

    dist_mag = (
        disturbance_step
        if disturbance_step is not None
        else default_disturbances.get(plant_id, 1.0)
    )

    res = plant.step(kp=kp, ki=ki, kd=kd, disturbance_step=dist_mag)

    return {
        "plant_id": res.plant_id,
        "gains": {"kp": res.kp, "ki": res.ki, "kd": res.kd},
        "disturbance_applied": dist_mag,
        "benchmark_score": res.metrics.performance_score,
        "itae_score": res.metrics.itae_score,
        "steady_state_error": res.metrics.steady_state_error,
        "is_stable": res.metrics.is_stable,
        "plot_ascii": res.plot_ascii,
    }


def main():
    """CLI entrypoint for loopbench."""
    mcp.run()


if __name__ == "__main__":
    main()
