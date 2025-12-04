import threading
import requests
import time

from typing import List, Callable, Tuple
from core.ports.sensor import SensorPort
from core.domain.samples import (
    RawSensorSample,
    SensorType,
    SensorSample,
)
from core.config import Config
from core.domain.pipeline.stage import Stage
from utils.stats_deque import TIMESTAMP_DIFF



class PhyphoxSensorAdapter(SensorPort):
    def __init__(
        self,
        cfg: Config,
        maxlen: int = 100,
        window: int = 50,
    ):
        self._ip = cfg.phyphox_ip()
        self._sensors = cfg.phyphox_sensors()
        self._running_ = True

        self.stage = Stage[SensorSample](
            sensors=self._sensors,
            maxlen=maxlen,
            window=window,
            stats=[TIMESTAMP_DIFF],
        )
        
        self._last_time = None
        self._measurement_types = [
            "gps_time",
            "gpsLat",
            "gpsLon",
            "gpsZ",
            "gpsV",
            "gpsDir",
        ]

    def stop(self):
        self._running_ = False

    def start(self):
        t = threading.Thread(target=self._get_data_loop, daemon=True)
        t.start()

    def put(self, sample: RawSensorSample):
        """Append sample to the corresponding sensor queue."""
        self.stage.put(sample.sensor, SensorSample.from_raw(sample))

    def get(self, sensor: SensorType) -> SensorSample | None:
        """Get oldest sample from specific sensor queue."""
        return self.stage.get(sensor)

    def get_buffer(self, sensor: SensorType) -> List[SensorSample]:
        """Return a snapshot of the internal queue."""
        return self.stage.get_buffer(sensor)

    def subscribe(self, sensor: SensorType, callback: Callable[[SensorSample], None]):
        """Observers will be called with each processed SensorSample."""
        self.stage.subscribe(sensor, callback)

    def _fech_data(self):
        if self._last_time is None:
            query = "&".join(f"{b}=full" for b in self._measurement_types)
        else:
            # incremental: use gps_time as reference
            query_parts = []
            for b in self._measurement_types:
                if b == "gps_time":
                    query_parts.append(f"{b}={self._last_time}")
                else:
                    # get values at same indices as gps_time
                    # syntax: threshold|reference_buffer
                    part = f"{b}={self._last_time}%7Cgps_time"
                    query_parts.append(part)
            query = "&".join(query_parts)

        url = f"http://{self._ip}/get?{query}"

        try:
            r = requests.get(url, timeout=2)
            r.raise_for_status()
            buf = r.json().get("buffer", {})

            # update last_time if new samples received
            times = buf.get("gps_time", {}).get("buffer", [])
            if times:
                self._last_time = max(times)

            return buf

        except Exception as e:
            return {}
        
    def _decode_data(self, buf) -> Tuple[SensorSample, SensorSample] | None:
        def get(b):
            return buf.get(b, {}).get("buffer", [])

        times = get("gps_time")
        lats = get("gpsLat")
        lons = get("gpsLon")
        zs = get("gpsZ")
        vs = get("gpsV")
        dirs = get("gpsDir")
    
        n = min(
            len(times),
            len(lats),
            len(lons),
            len(zs),
            len(vs),
            len(dirs),
        ) - 1
        
        if n == -1: return None
        
        timestamp = time.time_ns()
        lat = lats[n]
        lon = lons[n]
        alt = zs[n]  or 0.0
        heading = dirs[n] or 0.0
        v = vs[n] or 0.0
    
        latlon_sample = SensorSample(SensorType.GPS, timestamp, lat, lon, alt)
        velocity_sample = SensorSample(SensorType.ODOMETRY, timestamp, v, heading, 0)
        
        return (latlon_sample, velocity_sample)
    

    def _get_data_loop(self):
        n_no_data = 0
        while self._running_:
            data = self._fech_data()
            if data:
                samples = self._decode_data(data)
                if samples is not None:
                    n_no_data = 0
                    if SensorType.GPS in self._sensors:
                        self.stage.put(samples[0].sensor, samples[0])
                    if SensorType.ODOMETRY in self._sensors:
                        self.stage.put(samples[1].sensor, samples[1])
                else:
                    n_no_data += 1
                
                if n_no_data == 10: 
                    raise ValueError(f"Phyphox app not responding")
                
            time.sleep(1)
            
                    

