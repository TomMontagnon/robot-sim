from abc import ABC, abstractmethod


class AbstractRobotState(ABC):
    @abstractmethod
    def to_robot_frame(self) -> None:
        pass

    @abstractmethod
    def update(self) -> None:
        pass


class AbstractRobotCommand(ABC):
    pass


class AbstractRobotModel(ABC):
    @abstractmethod
    def dynamics(self, x: AbstractRobotState, u: AbstractRobotCommand) -> None:
        pass
