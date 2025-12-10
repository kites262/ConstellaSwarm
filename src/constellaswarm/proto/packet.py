from dataclasses import dataclass

from constellaswarm.proto import Vec3


@dataclass(slots=True)
class PeerInfo:
    timestamp: float
    observer: int
    peer: int
    reported_position: Vec3
    measured_distance: float
    measured_azimuth: float
    measured_elevation: float


@dataclass(slots=True)
class PerceptionFrame:
    timestamp: float
    measured_velocity: Vec3
    peers: list[PeerInfo]
