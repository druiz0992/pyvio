from abc import ABC, abstractmethod


class ReaderPort(ABC):
    @abstractmethod
    def read(self, size: int) -> bytes:
        """read bytes"""
        pass

    @abstractmethod
    def close(self):
        """close reder."""
        pass
