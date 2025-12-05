from abc import ABC, abstractmethod

class WriterPort(ABC):
    @abstractmethod
    def write(self, data: bytes):
        """ write bytes"""
        pass
    
    def close(self):
        """ Close writer. """
        pass