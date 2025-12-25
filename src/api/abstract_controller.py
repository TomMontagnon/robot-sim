from abc import ABC, abstractmethod

class AbstractController(ABC):
    @abstractmethod
    def compute() -> None:
        pass
