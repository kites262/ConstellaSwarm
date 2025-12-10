import time

import numpy as np
import rerun as rr
from loguru import logger
from utils.logger import setup_logger

from constellaswarm.helper.simulator import Simulator
from constellaswarm.model.drone import Drone, DroneAgent
from constellaswarm.proto.state import VehicleState

NUM_STEPS = 1000
TIME_STEP = 0.1


def main() -> None:
    setup_logger(level="DEBUG")
    rr.init("ConstellaSwarm Simulation", spawn=False)
    serve_uri = rr.serve_grpc()
    rr.serve_web_viewer(
        connect_to=serve_uri,
        open_browser=True,
        web_port=9090,
    )

    stimulator = Simulator(dt=TIME_STEP)

    num_steps = NUM_STEPS
    scale = 10.0
    initital_drones = [
        Drone(
            id=0,
            state=VehicleState(
                position=np.array([scale, scale, scale]),
                velocity=np.array([1.0, 1.0, 0.0]),
            ),
        ),
        Drone(
            id=1,
            state=VehicleState(
                position=np.array([scale, -scale, -scale]),
                velocity=np.array([1.0, 1.0, 0.0]),
            ),
        ),
        Drone(
            id=2,
            state=VehicleState(
                position=np.array([-scale, scale, -scale]),
                velocity=np.array([1.0, 1.0, 0.0]),
            ),
        ),
        Drone(
            id=3,
            state=VehicleState(
                position=np.array([-scale, -scale, scale]),
                velocity=np.array([1.0, 1.0, 0.0]),
            ),
        ),
    ]

    for i in range(len(initital_drones)):
        drone = initital_drones[i]
        stimulator.add_drone_agent(
            index=i,
            drone=drone,
            agent=DroneAgent(
                id=i,
                state=VehicleState(
                    position=drone.state.position.copy(),
                    velocity=drone.state.velocity.copy(),
                ),
            ),
        )

    for time_step in range(num_steps):
        step(stimulator)


def step(stimulator: Simulator) -> None:
    stimulator.step()

    curr_time = stimulator.time
    rr.set_time(timeline="time", timestamp=curr_time)

    for drone in stimulator.drones.values():
        rr.log(
            f"drone/{drone.id}",
            rr.Points3D(
                positions=[drone.state.position],
                colors=[(0, 0, 255)],
                radii=[0.3],
            ),
        )

    for agent in stimulator.agents.values():
        rr.log(
            f"agent/{agent.id}/position",
            rr.Points3D(
                positions=[agent.state.position],
                colors=[(255, 0, 0)],
                radii=[0.2],
            ),
        )

        rr.log(
            f"agent/{agent.id}/particles",
            rr.Points3D(
                positions=agent.particles,
                colors=[(180, 180, 180)],
                radii=[0.05],
            ),
        )

    metrics = evaluate_swarm_metrics_advanced(stimulator.drones, stimulator.agents)
    logger.info(
        f"Time: {curr_time:6.2f} | "
        f"MSE: {metrics['mse_total']:.4f} | "
        f"Trans: {metrics['mse_trans_inv']:.4f} | "
        f"Shape: {metrics['mse_shape_inv']:.4f}"
    )
    rr.log(
        "metrics/mse/total",
        rr.Scalars(
            scalars=[metrics["mse_total"]],
        ),
    )
    rr.log(
        "metrics/mse/translation_invariant",
        rr.Scalars(
            scalars=[metrics["mse_trans_inv"]],
        ),
    )
    rr.log(
        "metrics/mse/shape_invariant",
        rr.Scalars(
            scalars=[metrics["mse_shape_inv"]],
        ),
    )


def evaluate_swarm_metrics_advanced(drones: dict, agents: dict):
    ids = sorted(drones.keys())
    pos_true = np.array([drones[i].state.position for i in ids])
    pos_est = np.array([agents[i].state.position for i in ids])

    # 1. Level 1: 绝对误差 (包含 漂移 + 旋转 + 畸变)
    # 如果这个很大，说明你在地球坐标系里迷路了
    mse_total = np.mean(np.sum((pos_true - pos_est) ** 2, axis=1))

    # 2. Level 2: 编队一致性误差 (包含 旋转 + 畸变) -> 之前的 mse_formation
    # 这一步消除了平移漂移。如果这个很大，说明队形歪了(旋转)或者散了。
    c_true = np.mean(pos_true, axis=0)
    c_est = np.mean(pos_est, axis=0)
    mse_translation_invariant = np.mean(np.sum(((pos_true - c_true) - (pos_est - c_est)) ** 2, axis=1))

    # 3. Level 3: 纯形状误差 (只包含 畸变) -> Kabsch Error
    # 这一步消除了平移和旋转。这是算法的"底线"。
    # 只要这个值很小，说明你的 Swarm 算法里的"距离约束"是生效的，队形是刚性的。
    mse_shape_invariant = compute_kabsch_error(pos_true, pos_est)

    return {
        "mse_total": mse_total,
        "mse_trans_inv": mse_translation_invariant,  # 之前叫 mse_formation
        "mse_shape_inv": mse_shape_invariant,  # [NEW] 最小收敛情况
    }


def compute_kabsch_error(P_true: np.ndarray, P_est: np.ndarray) -> float:
    """
    计算消除平移和旋转后的形状误差 (Shape-only MSE)。

    Args:
        P_true: 真值点云 (N, 3)
        P_est: 估计值点云 (N, 3)

    Returns:
        float: 最优对齐后的 MSE
    """
    # 1. 去中心化 (Remove Translation)
    # 这一步其实就是之前的 mse_formation 做的事情
    center_true = np.mean(P_true, axis=0)
    center_est = np.mean(P_est, axis=0)

    P = P_true - center_true
    Q = P_est - center_est

    # 2. 计算协方差矩阵 H (Compute Covariance Matrix)
    # H = P.T @ Q
    H = np.dot(P.T, Q)

    # 3. 奇异值分解 SVD (Singular Value Decomposition)
    # H = U @ S @ Vt
    U, S, Vt = np.linalg.svd(H)

    # 4. 计算最优旋转矩阵 R (Optimal Rotation Matrix)
    # R = V @ U.T
    V = Vt.T
    R = np.dot(V, U.T)

    # [关键细节] 处理反射情况 (Reflection Case)
    # 有时候数学算出来的最优解是一个镜像(Reflection)，而不是旋转。
    # 我们需要检查行列式，如果 det(R) < 0，说明是镜像，需要修正。
    if np.linalg.det(R) < 0:
        V[:, 2] *= -1
        R = np.dot(V, U.T)

    # 5. 应用旋转 (Apply Rotation)
    # 将 P_true 旋转去匹配 P_est
    # 注意矩阵乘法顺序: (N,3) x (3,3)
    P_rotated = np.dot(P, R)

    # 6. 计算最终误差 (Calculate MSE)
    mse_shape = np.mean(np.sum((P_rotated - Q) ** 2, axis=1))

    return mse_shape


if __name__ == "__main__":
    main()
    time.sleep(10)
