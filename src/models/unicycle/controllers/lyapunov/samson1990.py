import numpy as np
from api.abstract_controller import AbstractController, compatible_with
from api.abstract_traj import AbstractTrajectory
from models.unicycle.model import UnicycleModel, UnicycleState, UnicycleCommand


@compatible_with(UnicycleModel)
class SamsonController(AbstractController):
    def __init__(self, k1: float = 10.0, k2: float = 1.0, k3: float = 10.0) -> None:
        self.k_1 = k1
        self.k_2 = k2
        self.k_3 = k3

    def compute(
        self, x: UnicycleState, traj: AbstractTrajectory, t: float
    ) -> UnicycleCommand:
        x_r, y_r, theta_r, v_r, w_r = traj.evaluate(t)

        ex, ey, etheta = x.to_robot_frame(x_r, y_r, theta_r)

        # Lyapunov-based control
        v = v_r * np.cos(etheta) + self.k_1 * ex
        w = w_r + self.k_2 * etheta + self.k_3 * v_r * np.sinc(etheta) * ey

        return UnicycleCommand(v, w)
