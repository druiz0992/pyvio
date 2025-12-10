from typing import Optional
import threading
import time

from pyvio.core.domain.params.stream_params import StreamParams
from pyvio.core.ports.sample import SampleType
from pyvio.adapters.readers import FileReader, UDPSocketReader
from pyvio.adapters.writers import FileWriter
from pyvio.core.ports.writer import WriterPort
from pyvio.core.ports.reader import ReaderPort
from pyvio.core.ports.sample import SamplePort
from pyvio.utils.stats_deque import TIMESTAMP_DIFF
from .stage import Stage


class Consumer:
    def __init__(
        self,
        stream_params: StreamParams,
        sample_cls: type[SamplePort],
        maxlen: int = 100,
        window: int = 50,
    ):

        self.sample_cls = sample_cls
        self.stage = Stage[sample_cls](
            sensors=SampleType.sample_list(),
            maxlen=maxlen,
            window=window,
            stats=[TIMESTAMP_DIFF],
        )
        self._running_ = True
        self._writer: Optional[WriterPort] = None
        self._delay: float = 0

        if stream_params.enable is False:
            raise ValueError("Streaming is not configured")

        # There are two modes of operation:
        # consume from socket. In this case, if file is defined, we will write data here as well
        # consume from file
        if stream_params.ip and stream_params.port:
            self._reader: ReaderPort = UDPSocketReader(
                stream_params.ip, stream_params.port
            )

            if stream_params.filename:
                self._writer: Optional[WriterPort] = FileWriter(stream_params.filename)
        elif stream_params.filename:
            self._reader: ReaderPort = FileReader(stream_params.filename)
            self._delay = 0.01
        else:
            raise ValueError(
                "Streaming is not configured. Missing 'filename' or socket"
            )

    def stop(self):
        self._running_ = False

    def start(self):
        t = threading.Thread(target=self._consume, daemon=False)
        t.start()

    def _consume(self):
        sample_size = self.sample_cls.sample_size()
        try:
            while self._running_:
                data = self._reader.read(sample_size)
                if not data:
                    continue

                if self._writer:
                    self._writer.write(data)

                try:
                    sample = self.sample_cls.from_bytes(data)
                    self.stage.put(sample.sample_type(), sample)
                except Exception as e:
                    print(f"Failed to parse sample: {e}")
                time.sleep(self._delay)
        finally:
            self._reader.close()
            if self._writer:
                self._writer.close()
