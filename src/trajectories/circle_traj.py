import numpy as np
from api.abstract_traj import AbstractTrajectory


class CircleTrajectory(AbstractTrajectory):
    def __init__(self, radius: float = 1.0, omega: float = 0.2) -> None:
        self.R = radius
        self.omega = omega

    def evaluate_pos(self, t: float) -> tuple:
        xd = self.R * np.cos(self.omega * t)
        yd = self.R * np.sin(self.omega * t)

        thetad = self.omega * t + np.pi / 2

        return xd, yd, thetad

    def evaluate_pos_vel(self, t: float) -> tuple:
        xd, yd, thetad = self.evaluate_pos(t)

        vd = self.R * self.omega
        wd = self.omega

        return xd, yd, thetad, vd, wd
