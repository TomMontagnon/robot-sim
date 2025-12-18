import numpy as np


class CircleTrajectory:
    def __init__(self, R=1.0, omega=0.2):
        self.R = R
        self.omega = omega

    def evaluatePos(self, t):
        xd = self.R * np.cos(self.omega * t)
        yd = self.R * np.sin(self.omega * t)

        thetad = self.omega * t + np.pi / 2

        return xd, yd, thetad

    def evaluatePosVel(self, t):
        xd, yd, thetad = self.evaluatePos(t)

        vd = self.R * self.omega
        wd = self.omega

        return xd, yd, thetad, vd, wd
