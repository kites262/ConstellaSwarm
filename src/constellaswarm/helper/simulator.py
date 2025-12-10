import numpy as np

from constellaswarm.model.drone import Drone, DroneAgent
from constellaswarm.proto.packet import PeerInfo, PerceptionFrame


class Simulator:
    drones: dict[int, Drone]
    agents: dict[int, DroneAgent]
    dt: float

    time: float

    env_wind_std: float = 1e-4
    sensor_imu_std: float = 1e-3
    sensor_range_std: float = 1e-2
    sensor_angle_std: float = 5e-3

    def __init__(self, dt: float = 0.1) -> None:
        self.time = 0.0
        self.agents = {}
        self.drones = {}
        self.dt = dt

    def add_drone_agent(self, index: int, drone: Drone, agent: DroneAgent) -> None:
        if drone.id != agent.id:
            raise ValueError("Drone ID and Agent ID must match.")
        self.drones[index] = drone
        self.agents[index] = agent

    def _update_physical_world(self) -> None:
        self.time += self.dt

        for drone in self.drones.values():
            wind = np.random.normal(0, self.env_wind_std, size=3)
            drone.state.velocity += wind
            drone.step(self.dt)

    def _calc_frames(self) -> dict[int, PerceptionFrame]:
        perceptions: dict[int, PerceptionFrame] = {}
        for agent in self.agents.values():
            peer_infos: list[PeerInfo] = []

            for other_agent in self.agents.values():
                if other_agent.id == agent.id:
                    continue

                distance = self.drones[agent.id].get_distance(self.drones[other_agent.id])
                distance = np.random.normal(distance, self.sensor_range_std)
                azimuth = self.drones[agent.id].get_azimuth(self.drones[other_agent.id])
                azimuth = np.random.normal(azimuth, self.sensor_angle_std)
                elevation = self.drones[agent.id].get_elevation(self.drones[other_agent.id])
                elevation = np.random.normal(elevation, self.sensor_angle_std)
                peer_pos = self.agents[other_agent.id].state.position

                peer_info = PeerInfo(
                    timestamp=self.time,
                    observer=agent.id,
                    peer=other_agent.id,
                    reported_position=peer_pos,
                    measured_distance=distance,
                    measured_azimuth=azimuth,
                    measured_elevation=elevation,
                )
                peer_infos.append(peer_info)

            perception = PerceptionFrame(
                timestamp=self.time,
                measured_velocity=self.drones[agent.id].state.velocity,
                peers=peer_infos,
            )
            perceptions[agent.id] = perception
        return perceptions

    def _update_cognitive_world(self, perceptions: dict[int, PerceptionFrame], dt: float) -> None:
        for agent in self.agents.values():
            perception = perceptions.get(agent.id)
            if perception is not None:
                agent.step(dt, perception)

    def step(self) -> None:
        # 更新物理世界
        self._update_physical_world()

        # 测算数据
        perceptions: dict[int, PerceptionFrame] = self._calc_frames()

        # 更新认知世界
        self._update_cognitive_world(perceptions, self.dt)
