from abc import ABC, abstractmethod


class AbstractAdapter(ABC):
    @abstractmethod
    def map(self, uc) -> None:
        pass
