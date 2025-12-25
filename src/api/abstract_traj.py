from abc import ABC, abstractmethod


class AbstractTrajectory(ABC):
    @abstractmethod
    def evaluate(self, t: float) -> tuple:
        pass
