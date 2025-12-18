from api.abstract_robot_model import AbstractRobotModel
import numpy as np


class UnicycleModel(AbstractRobotModel):
    def dynamics(self, x, u) -> np.ndarray:
        v, w = u
        theta = x[2]

        return np.array([v * np.cos(theta), v * np.sin(theta), w])

    def control_dim(self) -> int:
        return 2


def adapt_to_unicycle(v: float, w: float) -> np.ndarray:
    return np.array([v, w])
