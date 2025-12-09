from abc import ABC, abstractmethod
import numpy as np
from pyvio.core.domain.state import State

class StateIntegrator(ABC):
    @abstractmethod
    def integrate(self, state: State, w: np.ndarray, a: np.ndarray, dt: float) -> State:
        pass
