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


class CircleTrajectoryVariant:
    def __init__(self, radius: float = 1.0, omega: float = 0.2) -> None:
        self.R = radius
        self.omega = omega

    def evaluate_pos(self, t: float) -> tuple:
        # Position sur le cercle
        xd = self.R * np.cos(self.omega * t)
        yd = self.R * np.sin(self.omega * t)

        # Dérivées par rapport au temps
        xd_dot = -self.R * self.omega * np.sin(self.omega * t)
        yd_dot = self.R * self.omega * np.cos(self.omega * t)

        # Orientation tangentielle adaptée au modèle perpendiculaire
        thetad = np.arctan2(xd_dot, -yd_dot)  # theta tel que ẋ = v sinθ, ẏ = -v cosθ

        return xd, yd, thetad

    def evaluate_pos_vel(self, t: float) -> tuple:
        xd, yd, thetad = self.evaluate_pos(t)

        # Pour un cercle parfait, vd = R * omega
        vd = self.R * self.omega

        # Vitesse angulaire (constante pour un cercle)
        wd = self.omega

        return xd, yd, thetad, vd, wd
