from .abstract_robot_model import AbstractRobotModel
import numpy as np

#Command
# v = linear speed
# w = angular speed

#State
# x1 = linear x speed
# x2 = linear y speed
# x3 = angular speed

class UnicycleModel(AbstractRobotModel):
    def dynamics(self, x, u):
        v, w = u
        theta = x[2]

        return np.array([
            v * np.cos(theta),
            v * np.sin(theta),
            w
        ])

    def control_dim(self):
        return 2

