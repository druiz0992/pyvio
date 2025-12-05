from dataclasses import dataclass
from typing import Optional
import ipaddress
import re

DEFAULT_STREAM_ENABLE = False

@dataclass
class StreamParams:
    enable: bool = DEFAULT_STREAM_ENABLE
    filename: Optional[str] = None
    ip: Optional[str] = None
    port: Optional[int] = None

    @staticmethod
    def from_dict(stream_props: dict) -> "StreamParams":
        filename = None
        ip = None
        port = None
        enable = True
        
        filename = stream_props.get("filename")
        ip_port = stream_props.get("ip")
        
        if ip_port is not None:
            if not isinstance(ip_port, str):
                raise TypeError(f"ip_port must be a string, got {type(ip_port).__name__}")

            # Validate and split IP:port
            match = re.fullmatch(r"(\d{1,3}(?:\.\d{1,3}){3}):(\d{1,5})", ip_port)
            if not match:
                raise ValueError(f"ip_port '{ip_port}' is not in the format 'xxx.xxx.xxx.xxx:port'")
            ip_str, port_str = match.groups()
 
           # Validate IP
            try:
                ipaddress.IPv4Address(ip_str)
            except ipaddress.AddressValueError:
                raise ValueError(f"Invalid IPv4 address: {ip_str}")
            # Validate port
            port_int = int(port_str)
            if not (0 < port_int < 65536):
                raise ValueError(f"Port must be 1-65535, got {port_int}")

            ip = ip_str
            port = port_int
             
        if (not filename or not isinstance(filename, str)) and ip_port is None:
            enable = False
    
        return StreamParams(enable=enable, filename=filename, ip=ip, port=port)