import numpy as np
from api.abstract_controller import AbstractController
from api.abstract_traj import AbstractTrajectory
from models.unicycle.model import UnicycleModel, UnicycleState, UnicycleCommand


class KanayamaController(AbstractController):
    def __init__(self, kx: float = 1.0, ky: float = 2.0, ktheta: float = 2.0) -> None:
        self.k_x = kx
        self.k_y = ky
        self.k_theta = ktheta
        self.model = UnicycleModel

    def compute(
        self, x: UnicycleState, traj: AbstractTrajectory, t: float
    ) -> UnicycleCommand:
        x_r, y_r, theta_r, v_r, w_r = traj.evaluate(t)

        ex, ey, etheta = x.to_robot_frame(x_r, y_r, theta_r)

        # Lyapunov-based control
        v = v_r * np.cos(etheta) + self.k_x * ex
        w = w_r + v_r * (self.k_y * ey + self.k_theta * np.sin(etheta))

        return UnicycleCommand(v, w)

    def __str__(self) -> str:
        return f"Kanayama(kx:{self.k_x}, ky:{self.k_y}, ktheta:{self.k_theta})"
