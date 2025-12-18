from api.abstract_robot_model import AbstractRobotModel
import numpy as np


class BicycleModel(AbstractRobotModel):
    def __init__(self, length: float = 1.0) -> None:
        self.L = length

    def dynamics(self, x, u) -> list:
        v, delta = u
        theta = x[2]

        return np.array(
            [v * np.cos(theta), v * np.sin(theta), v / self.L * np.tan(delta)]
        )

    def control_dim(self) -> int:
        return 2

def adapt_to_bicycle(v: float, w: float) -> np.ndarray:
    raise NotImplementedError

