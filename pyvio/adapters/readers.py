import socket
import os
from typing import Optional

from pyvio.core.ports.reader import ReaderPort

class FileReader(ReaderPort):
    def __init__(self, filename: str, mode: str = "rb"):
        """
        File reader adapter.
            :param filename: Path to the file
            :param mode: File mode, default is read-binary
        """
        if not os.path.exists(filename):
            raise FileNotFoundError(f"File {filename} does not exist")
        self.file = open(filename, mode)
    
    def read(self, size: int = -1) -> bytes:
        """
        Read up to `size` bytes from the file. If size=-1, read entire file.
        """
        return self.file.read(size)
    
    def close(self):
        self.file.close()


class UDPSocketReader:
    def __init__(self, host: str, port: int):
        """
        UDP socket reader adapter.
            :param host: Local interface to bind (default 0.0.0.0)
            :param port: Local port to listen on
            :param buffer_size: Maximum UDP packet size to read
        """
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.addr)

    def read(self, buffer_size: int) -> bytes:
        """
        Read one UDP packet from the socket.
        """
        data, addr = self.sock.recvfrom(buffer_size)
        return data

    def close(self):
        self.sock.close()
