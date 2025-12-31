import numpy as np
from api.abstract_traj import AbstractTrajectory


class StraightTrajectory(AbstractTrajectory):
    def __init__(self, v: float, direc: list) -> None:
        self.v = v
        self.direc = direc

    def evaluate(self, t: float) -> tuple:
        xd = self.direc[0] * self.v * t
        yd = self.direc[1] * self.v * t

        thetad = np.arctan2(yd, xd)

        vd = self.v
        wd = 0.0

        return xd, yd, thetad, vd, wd
