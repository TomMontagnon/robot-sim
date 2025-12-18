from .abstract_adapter import AbstractAdapter
import numpy as np

class BicycleAdapter(AbstractAdapter):
    def __init__(self, L=1.0):
        self.L = L

    def map(self, uc):
        v, w = uc
        delta = np.arctan(self.L * w / (v + 1e-6))
        return np.array([v, delta])
