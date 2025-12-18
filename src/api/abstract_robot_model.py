from abc import ABC, abstractmethod


class AbstractRobotModel(ABC):
    @abstractmethod
    def dynamics(self, x, u) -> None:
        pass

    @abstractmethod
    def control_dim(self) -> None:
        pass
