import numpy as np
from api.abstract_traj import AbstractTrajectory

class StraightTrajectory(AbstractTrajectory):
    def __init__(self, v: float, direc: list, is_std : bool = True) -> None:
        self.v = v
        self.direc = direc
        self.is_std = is_std


    def evaluate(self, t : float) -> tuple:
        xd = self.direc[0] * self.v * t
        yd = self.direc[1] * self.v * t

        if self.is_std:
            thetad = np.arctan2(yd, xd)
        else:
            thetad = np.arctan2(self.direc[1], self.direc[0]) + np.pi/2

        vd = self.v
        wd = 0.0

        return xd, yd, thetad, vd, wd
