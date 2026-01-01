import numpy as np
from api.abstract_traj import AbstractTrajectory


class StraightTrajectory(AbstractTrajectory):
    def __init__(self, speed: float, direc: list) -> None:
        self.speed = speed
        self.direc = direc

    def evaluate(self, t: float) -> tuple:
        xd = self.direc[0] * self.speed * t
        yd = self.direc[1] * self.speed * t

        thetad = np.arctan2(yd, xd)

        vd = self.speed
        wd = 0.0

        return xd, yd, thetad, vd, wd
