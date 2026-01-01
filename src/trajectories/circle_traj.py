import numpy as np
from api.abstract_traj import AbstractTrajectory


class CircleTrajectory(AbstractTrajectory):
    def __init__(
        self, radius: float = 1.0, omega: float = 0.2, angle_offset: float = 0.0
    ) -> None:
        self.R = radius
        self.omega = omega
        self.angle_offset = angle_offset

    def evaluate(self, t: float) -> tuple:
        theta = self.omega * t + self.angle_offset

        xd = self.R * np.cos(theta)
        yd = self.R * np.sin(theta)

        thetad = theta + np.sign(self.omega) * np.pi / 2

        vd = self.R * np.abs(self.omega)
        wd = self.omega

        #offset
        xd -= self.R * np.cos(self.angle_offset)
        yd -= self.R * np.sin(self.angle_offset)

        #wrapped
        thetad_wrapped = np.arctan2(np.sin(thetad), np.cos(thetad))

        return xd, yd, thetad_wrapped, vd, wd
