"""Base abstractions and data contracts for LoopBench control targets."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple
from pydantic import BaseModel, Field

from loopbench.metrics import StepMetrics


class GainBounds(BaseModel):
    """Permissible range for PID gains to ensure safety."""
    kp_min: float = 0.0
    kp_max: float = 100.0
    ki_min: float = 0.0
    ki_max: float = 50.0
    kd_min: float = 0.0
    kd_max: float = 20.0


class PlantSpecs(BaseModel):
    """Static metadata and physical specs of a plant."""
    plant_id: str
    name: str
    description: str
    process_variable_unit: str
    control_effort_unit: str
    gain_bounds: GainBounds
    default_setpoint: float
    default_duration: float
    baseline_hints: str


class SimulationResult(BaseModel):
    """Output bundle generated from a single step-response evaluation."""
    plant_id: str
    kp: float
    ki: float
    kd: float
    setpoint: float
    duration: float
    metrics: StepMetrics
    plot_ascii: str
    time: List[float] = Field(default_factory=list, repr=False)
    output: List[float] = Field(default_factory=list, repr=False)
    control_effort: List[float] = Field(default_factory=list, repr=False)


class ControlTarget(ABC):
    """Abstract Base Class for all dynamic systems in LoopBench."""

    @abstractmethod
    def get_specs(self) -> PlantSpecs:
        """Return the physical specs, units, and safety bounds."""
        pass

    @abstractmethod
    def step(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float | None = None,
        duration: float | None = None,
        disturbance_step: float = 0.0,
    ) -> SimulationResult:
        """Simulate the closed-loop system response under PID control."""
        pass
