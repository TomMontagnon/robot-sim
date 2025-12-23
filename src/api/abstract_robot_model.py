from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class BasicRobotState:
    x: float
    y: float
    theta: float

    def to_robot_frame(
        self, x_r: float, y_r: float, theta_r: float, is_std: bool = True
    ) -> tuple:
        to_robot_frame = np.array(
            [
                [np.cos(self.theta), np.sin(self.theta), 0],
                [-np.sin(self.theta), np.cos(self.theta), 0],
                [0, 0, 1],
            ]
        )
        if not is_std:
            theta_r += np.pi/2

        e_world = np.array(
            [
                x_r - self.x,
                y_r - self.y,
                theta_r - self.theta,
            ]
        )
        if not is_std:
            e_world*=-1

        return to_robot_frame @ e_world

    def update(self, other: BasicRobotState, dt: float) -> None:
        self.x += other.x * dt
        self.y += other.y * dt
        self.theta += other.theta * dt


@dataclass
class BasicRobotCommand(ABC):
    v: float
    w: float


class AbstractRobotModel(ABC):
    @abstractmethod
    def dynamics(self, x: BasicRobotState, u: BasicRobotCommand) -> None:
        pass
