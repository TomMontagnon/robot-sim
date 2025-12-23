import numpy as np
from api.abstract_traj import AbstractTrajectory

class StraightTrajectory(AbstractTrajectory):
    def __init__(self, v: float, direc: list, is_variant : bool = False) -> None:
        self.v = v
        self.direc = direc
        self.is_variant = is_variant

    def evaluate_pos(self, t : float) -> tuple:
        xd = self.direc[0] * self.v * t
        yd = self.direc[1] * self.v * t

        if self.is_variant:
            thetad = np.arctan2(self.direc[1], self.direc[0]) + np.pi/2
        else:
            thetad = np.arctan2(yd, xd)

        return xd, yd, thetad

    def evaluate_pos_vel(self, t : float) -> tuple:
        xd, yd, thetad = self.evaluate_pos(t)

        vd = self.v
        wd = 0.0

        return xd, yd, thetad, vd, wd
