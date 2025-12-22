from __future__ import annotations
from api.abstract_robot_model import (
    AbstractRobotModel,
    AbstractRobotState,
    AbstractRobotCommand,
)
from dataclasses import dataclass
import numpy as np


@dataclass
class UnicycleState(AbstractRobotState):
    x: float
    y: float
    theta: float

    def to_robot_frame(self, x_r: float, y_r: float, theta_r: float) -> tuple:
        to_robot_frame = np.array(
            [
                [np.cos(self.theta), np.sin(self.theta), 0],
                [-np.sin(self.theta), np.cos(self.theta), 0],
                [0, 0, 1],
            ]
        )

        e_world = np.array(
            [
                x_r - self.x,
                y_r - self.y,
                theta_r - self.theta,
            ]
        )

        return to_robot_frame @ e_world

    def update(self, other : UnicycleState, dt : float) -> None:
        self.x += other.x * dt
        self.y += other.y * dt
        self.theta += other.theta * dt

@dataclass
class UnicycleCommand(AbstractRobotCommand):
    v: float
    w: float


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
