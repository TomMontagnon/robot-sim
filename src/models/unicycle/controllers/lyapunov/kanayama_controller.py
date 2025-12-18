import numpy as np
from api.abstract_controller import AbstractController,compatible_with
from models.unicycle.model import UnicycleModel

@compatible_with(UnicycleModel)
class KanayamaController(AbstractController):
    def __init__(self, kx=1.0, ky=2.0, ktheta=2.0) -> None:
        self.kx = kx
        self.ky = ky
        self.ktheta = ktheta

    def compute(self, x, traj, t):
        xd, yd, thetad, vd, wd = traj.evaluate_pos_vel(t)

        dx = xd - x[0]
        dy = yd - x[1]
        theta = x[2]

        # Errors in robot frame
        ex =  np.cos(theta) * dx + np.sin(theta) * dy
        ey = -np.sin(theta) * dx + np.cos(theta) * dy
        etheta = thetad - theta

        # Lyapunov-based control
        v = vd * np.cos(etheta) + self.kx * ex
        w = wd + self.ky * vd * ey + self.ktheta * np.sin(etheta)

        return np.array([v, w])
