from abc import ABC, abstractmethod


class AbstractTrajectory(ABC):
    @abstractmethod
    def evaluate_pos(self, t: float) -> tuple:
        pass

    @abstractmethod
    def evaluate_pos_vel(self, t: float) -> tuple:
        pass
