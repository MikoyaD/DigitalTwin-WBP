"""Extension contracts only: no geometry optimizer, retimer, refiner or ROS code."""
from typing import Protocol, Sequence
from .contracts import PathRecord, PhasePath, LimitSet, TrajectoryRecord


class GeometryBuilder(Protocol):
    def build(self, path: PathRecord) -> Sequence[PhasePath]: ...


class TimeParameterizer(Protocol):
    def parameterize(self, phase: PhasePath, limits: LimitSet) -> TrajectoryRecord: ...


class JerkRefiner(Protocol):
    def refine(self, phase: PhasePath, trajectory: TrajectoryRecord,
               limits: LimitSet) -> TrajectoryRecord: ...


class InputAdapter(Protocol):
    def capture(self, raw_response: object, metadata: dict) -> PathRecord: ...
