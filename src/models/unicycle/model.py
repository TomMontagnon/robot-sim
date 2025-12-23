from __future__ import annotations
from api.abstract_robot_model import (
    AbstractRobotModel,
    BasicRobotState,
    BasicRobotCommand,
)
import numpy as np


class UnicycleState(BasicRobotState):
    pass


class UnicycleCommand(BasicRobotCommand):
    pass


class UnicycleModel(AbstractRobotModel):
    def dynamics(self, x: UnicycleState, u: UnicycleCommand) -> UnicycleState:
        x_dot = u.v * np.cos(x.theta)
        y_dot = u.v * np.sin(x.theta)
        theta_dot = u.w

        return UnicycleState(x_dot, y_dot, theta_dot)


class UnicycleVariantModel(AbstractRobotModel):
    def dynamics(self, x: UnicycleState, u: UnicycleCommand) -> UnicycleState:
        x_dot = u.v * np.sin(x.theta)
        y_dot = -u.v * np.cos(x.theta)
        theta_dot = u.w

        return UnicycleState(x_dot, y_dot, theta_dot)


def adapt_to_unicycle(v: float, w: float) -> UnicycleCommand:
    return UnicycleCommand(v, w)
