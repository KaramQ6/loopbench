"""Registry and factory for all dynamic plants available in LoopBench."""

from __future__ import annotations

from typing import Dict, List
from loopbench.base import ControlTarget, PlantSpecs
from loopbench.plants.dc_motor import DCMotorPlant
from loopbench.plants.thermal import ThermalPlant
from loopbench.plants.drone import DroneAltitudePlant

_PLANTS: Dict[str, ControlTarget] = {
    "dc_motor": DCMotorPlant(),
    "thermal": ThermalPlant(),
    "drone": DroneAltitudePlant(),
}


def get_plant(plant_id: str) -> ControlTarget:
    """Retrieve a plant instance by its unique identifier."""
    if plant_id not in _PLANTS:
        valid_ids = list(_PLANTS.keys())
        raise ValueError(f"Unknown plant_id '{plant_id}'. Available plants: {valid_ids}")
    return _PLANTS[plant_id]


def list_available_plants() -> List[PlantSpecs]:
    """Return physical specifications for all registered simulation models."""
    return [plant.get_specs() for plant in _PLANTS.values()]
