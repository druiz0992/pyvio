from pyvio.core.domain.samples import SensorSample
from pyvio.adapters.readers import UDPSocketReader
from pyvio.core.config import Config


def main():
    cfg = Config("./configs/receiver.yaml")
    stream_params = cfg.stream_params()
    sample_size = SensorSample.sample_size()
    
    host = stream_params.ip or "0.0.0.0"
    port = stream_params.port
   
    if port:
       reader = UDPSocketReader(host, port)
    else:
       raise FileNotFoundError(f"No port assigned")
   
  
    while True:
        data = reader.read(sample_size)
    
        sample = SensorSample.from_bytes(data)
        print(sample)
        
        
if __name__ == "__main__":
    main()