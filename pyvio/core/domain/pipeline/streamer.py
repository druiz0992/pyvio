from collections import deque
from typing import List, Dict, Optional
import threading

from pyvio.core.domain.params.stream_params import StreamParams
from pyvio.core.ports.sample import SampleType, SamplePort
from pyvio.adapters.writers import FileWriter, UDPSocketWriter
from pyvio.core.ports.writer import WriterPort
from .stage import Stage

class Streamer:
    def __init__(self, stages: List[Stage], stream_params: StreamParams, maxlen: int = 100):
        self.stages = stages
        self.buffers: Dict[SampleType, deque] = {}
        self._running_ = True
        
        if stream_params.enable is False:
            raise ValueError("Streaming is not configured")
        
        self._file_writer: Optional[WriterPort] = FileWriter(stream_params.filename) \
            if stream_params.filename is not None else None
        self._socket_writer: Optional[WriterPort] = UDPSocketWriter(stream_params.ip, stream_params.port) \
            if stream_params.ip and stream_params.port is not None else None
            
        self._writers: List[WriterPort] = [
            w for w in [self._file_writer, self._socket_writer] if w is not None
        ]

        # Initialize buffers for all sensors across all stages
        for stage in stages:
            for sensor in stage.queues.keys():
                if sensor not in self.buffers:
                    self.buffers[sensor] = deque(maxlen=maxlen)
                    # Subscribe to update buffer whenever a new sample arrives
                    #stage.subscribe(sensor, lambda sample, s=sensor: self.buffers[s].append(sample))
                    stage.subscribe(sensor, lambda sample, s=sensor: self.append(s, sample))
                    
    def append(self, s, sample):
        self.buffers[s].append(sample)
                    
    def stop(self):
        self._running_ = False

    def start(self):
        t = threading.Thread(target=self._stream, daemon=True)
        t.start()
    
    def _stream(self):
        while self._running_:
            for buf in self.buffers.values():
                if buf:
                    sample: SamplePort = buf.popleft()
                    data = sample.to_bytes()
                    for w in self._writers:
                        w.write(data)
                        
        for w in self._writers:
            w.close()

