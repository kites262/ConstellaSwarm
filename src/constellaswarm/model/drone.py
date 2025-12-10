from dataclasses import dataclass, field

import numpy as np

from constellaswarm.proto.packet import PerceptionFrame
from constellaswarm.proto.state import VehicleState


@dataclass(slots=True)
class Drone:
    id: int
    state: VehicleState

    def step(self, dt: float) -> None:
        """Update the physical drone's state over a time step.

        Args:
            dt (float): Time step duration.
        """
        self.state.position += self.state.velocity * dt

    def get_distance(self, other: "Drone") -> float:
        """Measure the distance to another drone.

        Args:
            other (Drone): The other drone.

        Returns:
            float: Euclidean distance to the other drone.
        """
        delta = other.state.position - self.state.position
        dist = float(np.linalg.norm(delta))
        return dist

    def get_azimuth(self, other: "Drone") -> float:
        """Measure the azimuth angle to another drone in the horizontal plane.

        Args:
            other (Drone): The other drone.
        Returns:
            float: Azimuth angle in radians.
        """
        delta = other.state.position - self.state.position
        return float(np.arctan2(delta[1], delta[0]))  # Angle in the XY-plane

    def get_elevation(self, other: "Drone") -> float:
        """Measure the elevation angle to another drone.

        Args:
            other (Drone): The other drone.
        Returns:
            float: Elevation angle in radians.
        """
        delta = other.state.position - self.state.position
        horizontal_distance = np.linalg.norm(delta[:2])
        return float(np.arctan2(delta[2], horizontal_distance))  # Angle from horizontal plane


@dataclass(slots=True)
class DroneAgent:
    id: int
    state: VehicleState  # Agent perceived state

    NUM_PARTICLES: int = 1000
    particles: np.ndarray = field(init=False)
    weights: np.ndarray = field(init=False)

    process_noise_std: float = 0.05
    measurement_noise_std: float = 0.1

    def __post_init__(self) -> None:
        self.particles = np.random.normal(
            loc=self.state.position,
            scale=0.1,
            size=(self.NUM_PARTICLES, 3),
        )
        self.weights = np.ones(self.NUM_PARTICLES) / self.NUM_PARTICLES

    def step(self, dt: float, perception: PerceptionFrame) -> None:
        """Update the agent's perceived state over a time step."""

        # ==========================
        # 1. 预测 (Prediction)
        # ==========================
        # 利用 IMU 速度驱动粒子
        displacement = perception.measured_velocity * dt

        process_noise = np.random.normal(
            loc=0.0,
            scale=self.process_noise_std,
            size=(self.NUM_PARTICLES, 3),
        )
        self.particles += displacement + process_noise

        # ==========================
        # 2. 修正 (Correction)
        # ==========================
        if perception.peers:
            # 暂存似然度乘积
            cumulative_likelihoods = np.ones(self.NUM_PARTICLES)

            for peer in perception.peers:
                # 获取数据
                neighbor_pos = peer.reported_position
                z_dist = peer.measured_distance

                # 计算所有粒子到该邻居位置的距离
                # self.particles: (N, 3), neighbor_pos: (3,) -> (N,)
                d_pred = np.linalg.norm(self.particles - neighbor_pos, axis=1)

                # 高斯似然计算
                error = z_dist - d_pred
                # 使用 measurement_noise_std 作为标准差
                likelihood = np.exp(-(error**2) / (2 * self.measurement_noise_std**2))

                # 累乘权重 (Bayes Update)
                cumulative_likelihoods *= likelihood

            # 更新当前权重
            self.weights *= cumulative_likelihoods

            # 归一化与异常处理
            w_sum = np.sum(self.weights)
            if w_sum > 1e-12:
                self.weights /= w_sum
            else:
                # 粒子贫化/跟丢保护：重置为均匀分布
                self.weights.fill(1.0 / self.NUM_PARTICLES)

            # ==========================
            # 3. 重采样 (Resampling)
            # ==========================
            # 计算有效粒子数 (Effective Sample Size)
            n_eff = 1.0 / np.sum(self.weights**2)

            # 当有效粒子少于一半时，进行重采样
            if n_eff < self.NUM_PARTICLES / 2:
                self._resample()

        # ==========================
        # 4. 状态估计 (Estimation)
        # ==========================
        # 计算加权平均位置
        estimated_pos = np.average(self.particles, axis=0, weights=self.weights)

        self.state.position = estimated_pos
        self.state.velocity = perception.measured_velocity

    def _resample(self) -> None:
        """执行重采样，保留高权重粒子"""
        # 使用 Numpy 的 choice 进行重采样
        indices = np.random.choice(
            self.NUM_PARTICLES,
            size=self.NUM_PARTICLES,
            p=self.weights,
        )

        # 复制被选中的粒子
        self.particles = self.particles[indices]

        # 重置权重为均匀分布
        self.weights.fill(1.0 / self.NUM_PARTICLES)
