"""LoopBench Evaluation & Leaderboard Runner.

Runs baseline classical tuners against target plants and prints a Markdown Leaderboard.
Can be extended with API calls to evaluate LLM-generated PID gains.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Any, List

# Ensure utf-8 output on Windows consoles
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

# Ensure src is on sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from loopbench.plants import list_available_plants, get_plant


# Baseline gains for each plant
BASELINES = {
    "dc_motor": {"kp": 2.5, "ki": 5.0, "kd": 0.05},
    "thermal": {"kp": 2.0, "ki": 0.3, "kd": 0.5},
    "drone": {"kp": 8.0, "ki": 3.0, "kd": 6.0},
}


def run_evaluations() -> List[Dict[str, Any]]:
    plants = list_available_plants()
    results = []

    for spec in plants:
        plant = get_plant(spec.plant_id)
        gains = BASELINES.get(spec.plant_id, {"kp": 1.0, "ki": 0.0, "kd": 0.0})
        
        # Test with baseline recommended gains
        res = plant.step(
            kp=gains["kp"],
            ki=gains["ki"],
            kd=gains["kd"],
            setpoint=spec.default_setpoint,
            duration=spec.default_duration,
        )

        results.append({
            "plant_id": spec.plant_id,
            "name": spec.name,
            "gains": f"Kp={gains['kp']}, Ki={gains['ki']}, Kd={gains['kd']}",
            "overshoot_pct": f"{res.metrics.overshoot_pct:.1f}%",
            "settling_s": f"{res.metrics.settling_time:.3f}s" if res.metrics.settling_time else "N/A",
            "itae": f"{res.metrics.itae_score:.2f}",
            "score": f"{res.metrics.performance_score:.1f}/100",
            "stable": "✅ Stable" if res.metrics.is_stable else "❌ Unstable",
        })

    return results


def print_leaderboard(results: List[Dict[str, Any]]):
    print("\n# 🏆 LoopBench Baseline Performance Summary\n")
    print("| Plant | Controller / Gains | Overshoot | Settling Time | ITAE | Quality Score | Stability |")
    print("|---|---|---|---|---|---|---|")
    for r in results:
        print(f"| {r['name']} | `{r['gains']}` | {r['overshoot_pct']} | {r['settling_s']} | {r['itae']} | {r['score']} | {r['stable']} |")
    print("\nRun completed successfully.\n")


if __name__ == "__main__":
    results = run_evaluations()
    print_leaderboard(results)
