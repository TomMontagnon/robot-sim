from abc import ABC, abstractmethod

class AbstractController(ABC):
    @abstractmethod
    def compute() -> None:
        pass


def compatible_with(*model_classes):
    """
    Décorateur pour déclarer explicitement
    quels modèles un contrôleur peut gérer.
    """
    def wrapper(cls):
        cls.compatible_models = model_classes
        return cls
    return wrapper
