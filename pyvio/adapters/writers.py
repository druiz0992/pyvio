import socket
import os

from pyvio.core.ports.writer import WriterPort

class FileWriter(WriterPort):
    def __init__(self, filename: str, mode: str = "wb"):
        """
        File writer adapter.
            :param filename: Path to the file
            :param mode: File mode, default is write-binary
        """
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        self.file = open(filename, mode)
    
    def write(self, data: bytes):
        self.file.write(data)
        self.file.flush()  # ensure it's written immediately
    
    def close(self):
        self.file.close()

class UDPSocketWriter(WriterPort):
    def __init__(self, host: str, port: int):
        """
        UDP socket writer adapter.
            :param host: Target hostname or IP
            :param port: Target port
        """
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def write(self, data: bytes):
        self.sock.sendto(data, self.addr)

    def close(self):
        self.sock.close()
