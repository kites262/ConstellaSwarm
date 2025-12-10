from dataclasses import dataclass, field

import numpy as np

from constellaswarm.proto import Vec3


@dataclass(slots=True)
class VehicleState:
    position: Vec3 = field(default_factory=lambda: np.zeros(3))
    velocity: Vec3 = field(default_factory=lambda: np.zeros(3))
