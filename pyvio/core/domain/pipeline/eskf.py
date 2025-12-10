from typing import List, Dict, Optional
import threading
import time
from collections import deque

from pyvio.core.domain.sample import IMUSample
from pyvio.core.ports.integrator import StateIntegrator
from pyvio.core.ports.sample import SampleType, SamplePort
from pyvio.core.domain.state import State
from pyvio.core.services.eskf import ESKF, StaticIMUInit
from pyvio.core.services.integrator import EulerIntegrator
from pyvio.utils.stats_deque import TIMESTAMP_DIFF
from .stage import Stage


class ESKFStage:
    def __init__(
        self,
        stages: List[Stage],
        integrator_type: Optional[type[StateIntegrator]] = None,
        maxlen=100,
        window=50,
        init_time_s: float = 5.0
    ):
        self.input_stages = stages   # imu, gps and odometry
        self.buffers: Dict[SampleType, deque] = {}
        self._running_ = True
        self.output_stage = Stage[State](
            sensors=[SampleType.STATE],
            maxlen=maxlen,
            window=window,
            stats=[TIMESTAMP_DIFF],
        )
        
        self._handlers = {
            SampleType.IMU: self._handle_imu,
            SampleType.GPS: self._handle_gps,
            SampleType.ODOMETRY: self._handle_odom,
        }
        integrator_type = EulerIntegrator if integrator_type is None else integrator_type
        self._eskf = ESKF(state=None, integrator_type = integrator_type)
        self._imu_init = StaticIMUInit(init_time_s=init_time_s)
        self._imu_inited = False

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
        t = threading.Thread(target=self._estimator, daemon=False)
        t.start()

    def _estimator(self):
        while self._running_:
            for sensor, buf in self.buffers.items():
                if not buf:
                    continue
                
                sample: SamplePort = buf.popleft()
                
                handler = self._handlers.get(sensor)
                
                if handler is not None:
                    state = handler(sample)
                    if state is not None:
                       self.output_stage.put(SampleType.STATE, state)
                       
            time.sleep(0.0005)
                       
    def _handle_imu(self, sample: IMUSample):
        if not self._imu_inited:
            if self._imu_init.add_sample(sample):
                self._imu_init.set_eskf_initial_state(self._eskf)
                self._imu_inited = True
            return None
        
        self._eskf.predict(
            w_meas=sample.gyro,
            a_meas=sample.acc,
            timestamp=sample.timestamp,
        )
    
        return self._eskf.x_nom
    
    def _handle_gps(self, sample):
        # sample.value must be np.array([lat, lon, alt])
        if not self._imu_inited:
            return None
        self._eskf.update_position(sample.as_array(), lla=True)
        return self._eskf.x_nom
    
    def _handle_odom(self, sample):
        # sample.value should be 3-vector of velocity in ENU or body->ENU converted
        if not self._imu_inited:
            return None
        self._eskf.update_velocity(sample.as_array())
        return self._eskf.x_nom


                       
                    
            
