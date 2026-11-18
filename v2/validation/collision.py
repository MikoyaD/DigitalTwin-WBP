"""Only a collision contract. No PlanningScene implementation in phase I."""
from dataclasses import dataclass
from typing import Optional, Protocol
import numpy as np
from ..core.contracts import StateSchema, Status, array, check_hash, require


@dataclass(frozen=True)
class CollisionRequest:
    q: np.ndarray
    schema: StateSchema
    scene_hash: Optional[str]
    model_hash: str
    interpolation: str

    def __post_init__(self):
        require(isinstance(self.schema, StateSchema), 'MISSING_METADATA', 'collision StateSchema')
        q = np.asarray(self.q)
        require(q.ndim == 2 and len(q) > 0, 'DIMENSION', 'collision states')
        object.__setattr__(self, 'q', array(q, (len(q), 9), 'collision q'))
        check_hash(self.model_hash, 'model_hash')
        if self.scene_hash is not None:
            check_hash(self.scene_hash, 'scene_hash')
        require(bool(self.interpolation), 'MISSING_METADATA', 'collision interpolation convention')


@dataclass(frozen=True)
class CollisionResult:
    status: Status
    reason: str
    scene_hash: Optional[str]
    checked_states: int

    def __post_init__(self):
        require(isinstance(self.status, Status), 'COLLISION_STATUS', 'use Status')
        if self.status == Status.PASS:
            require(self.scene_hash is not None and self.checked_states > 0,
                    'COLLISION_EVIDENCE_MISSING', 'PASS requires scene evidence and checked states')
            check_hash(self.scene_hash, 'scene_hash')


class CollisionChecker(Protocol):
    def check(self, request: CollisionRequest) -> CollisionResult: ...


class UnknownCollisionChecker:
    def check(self, request):
        return CollisionResult(Status.UNKNOWN, 'NO_PLANNING_SCENE_BACKEND', request.scene_hash, 0)
