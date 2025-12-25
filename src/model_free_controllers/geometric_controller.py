import numpy as np
from collections.abc import Callable
from api.abstract_controller import AbstractController
from api.abstract_traj import AbstractTrajectory
from api.abstract_robot_model import BasicRobotState, BasicRobotCommand

class GeometricController(AbstractController):
    def __init__(
        self,
        k_v: float,
        k_w: float,
        input_adapter: Callable,
        output_adapter: Callable,
        is_feedworwarded: bool = True,
        is_robot_framed: bool = True,
    ) -> None:
        self.is_feedforwarded = is_feedworwarded
        self.is_robot_framed = is_robot_framed
        self.k_v = k_v
        self.k_w = k_w
        self.input_adapter = input_adapter
        self.output_adapter = output_adapter

    def compute(
        self, raw_x: BasicRobotState, traj: AbstractTrajectory, t: float
    ) -> BasicRobotCommand:
        x = self.input_adapter(raw_x)
        x_r, y_r, theta_r, v_r, w_r = traj.evaluate(t)
        if not self.is_feedforwarded:
            v_r, w_r = 0, 0

        dx = x_r - x.x
        dy = y_r - x.y

        theta = x.theta
        etheta = theta_r - theta

        if self.is_robot_framed:
            ex_robot = np.cos(theta) * dx + np.sin(theta) * dy
            ey_robot = -np.sin(theta) * dx + np.cos(theta) * dy

            var_fb_v = ex_robot
            var_fb_w = ey_robot + etheta
        else:
            dist = np.sqrt(dx**2 + dy**2)
            var_fb_v = dist
            var_fb_w = etheta

        v = v_r + self.k_v * var_fb_v
        w = w_r + self.k_w * var_fb_w

        return self.output_adapter(v, w)  # commande canonique
