import numpy as np
from api.abstract_controller import AbstractController, compatible_with
from api.abstract_traj import AbstractTrajectory
from models.unicycle.model import UnicycleVariantModel, UnicycleState, UnicycleCommand


@compatible_with(UnicycleVariantModel)
class M2parController(AbstractController):
    def __init__(self, k1: float = 1.0, k2: float = 1.0, k3: float = 1.0) -> None:
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

    def compute(
        self, x: UnicycleState, traj: AbstractTrajectory, t: float
    ) -> UnicycleCommand:
        # Trajectoire de référence
        x_r, y_r, theta_r, v_r, w_r = traj.evaluate_pos_vel(t)

        e1, e2, e3 = x.to_robot_frame(x_r, y_r, theta_r, is_std=False)

        # Calcul de la loi de commande perpendiculaire
        m1 = -self.k2 * e2
        m2 = self.k1 * v_r * e1 * np.sinc(e3 / np.pi) + self.k3 * e3

        v = v_r * np.cos(e3) - m1
        w = w_r - m2

        # print("state :")
        # print(x.x, x.y, x.theta)
        # print("consigne")
        # print(x_r, y_r, theta_r, v_r, w_r)
        # print("sortie")
        # print(v, w)
        # input()
        return UnicycleCommand(v, w)
