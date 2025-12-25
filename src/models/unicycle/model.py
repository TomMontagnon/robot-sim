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
    is_std = True

    def dynamics(self, x: UnicycleState, u: UnicycleCommand) -> UnicycleState:
        x_dot = u.v * np.cos(x.theta)
        y_dot = u.v * np.sin(x.theta)
        theta_dot = u.w

        return UnicycleState(x_dot, y_dot, theta_dot)

    def adapt_input(self, x: UnicycleState) -> UnicycleState:
        return x

    def adapt_output(self, v: float, w: float) -> UnicycleCommand:
        return UnicycleCommand(v, w)


class UnicycleM2Model(AbstractRobotModel):
    is_std = False

    def dynamics(self, x: UnicycleState, u: UnicycleCommand) -> UnicycleState:
        x_dot = u.v * np.sin(x.theta)
        y_dot = -u.v * np.cos(x.theta)
        theta_dot = u.w

        return UnicycleState(x_dot, y_dot, theta_dot)

    def adapt_input(self, x: UnicycleState) -> UnicycleState:
        return UnicycleState(x.x, x.y, x.theta - np.pi/2)

    def adapt_output(self, v: float, w: float) -> UnicycleCommand:
        return UnicycleCommand(v, w)
