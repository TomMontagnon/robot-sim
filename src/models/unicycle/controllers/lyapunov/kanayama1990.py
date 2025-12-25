import numpy as np
from api.abstract_controller import AbstractController
from api.abstract_traj import AbstractTrajectory
from models.unicycle.model import UnicycleModel, UnicycleState, UnicycleCommand


class KanayamaController(AbstractController):
    def __init__(self, kx: float = 1.0, ky: float = 2.0, ktheta: float = 2.0) -> None:
        self.kx = kx
        self.ky = ky
        self.ktheta = ktheta
        self.model = UnicycleModel

    def compute(
        self, x: UnicycleState, traj: AbstractTrajectory, t: float
    ) -> UnicycleCommand:
        x_r, y_r, theta_r, v_r, w_r = traj.evaluate(t)

        ex, ey, etheta = x.to_robot_frame(x_r, y_r, theta_r)

        # Lyapunov-based control
        v = v_r * np.cos(etheta) + self.kx * ex
        w = w_r + v_r * (self.ky * ey + self.ktheta * np.sin(etheta))

        return UnicycleCommand(v, w)
