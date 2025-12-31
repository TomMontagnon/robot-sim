import numpy as np
from api.abstract_traj import AbstractTrajectory


class CircleTrajectory(AbstractTrajectory):
    def __init__(self, radius: float = 1.0, omega: float = 0.2) -> None:
        self.R = radius
        self.omega = omega

    def evaluate(self, t: float) -> tuple:
        xd = self.R * np.cos(self.omega * t)
        yd = self.R * np.sin(self.omega * t)

        thetad = self.omega * t + np.sign(self.omega) * np.pi / 2

        vd = self.R * np.abs(self.omega)
        wd = self.omega

        return xd, yd, thetad, vd, wd
