import numpy as np
from .abstract_controller import AbstractController


class GeometricController(AbstractController):
    def __init__(
        self, is_feedworwarded: bool, is_decoupled: bool, k_v: float, k_w: float
    ) -> None:
        self.is_feedforwarded = is_feedworwarded
        self.is_decoupled = is_decoupled
        self.k_v = k_v
        self.k_w = k_w

    def compute(self, x, traj, t):
        if self.is_feedforwarded:
            xd, yd, thetad, vd, wd = traj.evaluatePosVel(t)
        else:
            xd, yd, thetad = traj.evaluatePos(t)
            vd, wd = 0, 0

        dx = xd - x[0]
        dy = yd - x[1]

        theta = x[2]
        etheta = thetad - theta

        if self.is_decoupled:
            ex_robot = np.cos(theta) * dx + np.sin(theta) * dy
            ey_robot = -np.sin(theta) * dx + np.cos(theta) * dy

            var_fb_v = ex_robot
            var_fb_w = ey_robot + etheta
        else:
            dist = np.sqrt(dx**2 + dy**2)
            var_fb_v = dist
            var_fb_w = etheta

        v = vd + self.k_v * var_fb_v
        w = wd + self.k_w * var_fb_w

        return np.array([v, w])  # commande canonique
