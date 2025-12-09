from typing import List
import threading
import time
from collections import deque

from pyvio.core.ports.sample import SampleType
from pyvio.core.domain.sample import IMUSample
from pyvio.core.ports.interpolator import IMUInterpolator
from pyvio.utils.stats_deque import TIMESTAMP_DIFF
from .stage import Stage


DELAY = 0.0001
class Interpolator:
    def __init__(self, stage: Stage, sensors: List[SampleType], interpolator_type: type[IMUInterpolator], maxlen=100, window = 50):
        self.input_stage = stage
        self._sensors = sensors
        self.buffers = {s: deque(maxlen=maxlen) for s in sensors}
        self._running_ = True
        self.output_stage = Stage[IMUSample](
            sensors=[SampleType.IMU],
            maxlen=maxlen,
            window=window,
            stats=[TIMESTAMP_DIFF],
        )
        
        self.interpolator = interpolator_type(self.buffers[SampleType.ACCELEROMETER], self.buffers[SampleType.GYROSCOPE])

        for s in self._sensors:
            self.input_stage.subscribe(s, lambda sample, s=s: self.buffers[s].append(sample))

    def stop(self):
        self._running_ = False

    def start(self):
        t = threading.Thread(target=self._interpolate, daemon=False)
        t.start()

    def _interpolate(self):
        acc_buf = self.buffers[SampleType.ACCELEROMETER]
        gyr_buf = self.buffers[SampleType.GYROSCOPE]
        
        last_t_out = None
        
        while self._running_:
            
            if len(acc_buf) < 2 or len(gyr_buf) < 2:
                time.sleep(DELAY)  
                continue
            
            t_acc0 = acc_buf[0].timestamp
            t_gyr0 = gyr_buf[0].timestamp
            t_out = max(t_acc0, t_gyr0)
            
            if last_t_out is not None and t_out <= last_t_out:
                time.sleep(DELAY)
                continue

            acc_interp = self.interpolator.interpolate(acc_buf, t_out)
            gyr_interp = self.interpolator.interpolate(gyr_buf, t_out)
            
            if acc_interp is None or gyr_interp is None:
                time.sleep(DELAY)  
                continue
                
            imu_sample = IMUSample(SampleType.IMU, t_out, acc_interp.as_array(), gyr_interp.as_array())

            # Push synchronized IMU to output stage
            self.output_stage.put(SampleType.IMU, imu_sample)
            
            last_t_out = t_out

            # Drop used samples 
            while len(acc_buf) > 2 and acc_buf[1].timestamp <= t_out:
                acc_buf.popleft()

            while len(gyr_buf) > 2 and gyr_buf[1].timestamp <= t_out:
                gyr_buf.popleft()
            
