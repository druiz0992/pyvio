from typing import List
import threading
import time
from collections import deque

from pyvio.core.ports.integrator import StateIntegrator
from pyvio.core.ports.sample import SampleType
from pyvio.core.domain.sample import IMUSample
from pyvio.core.domain.state import State
from pyvio.utils.stats_deque import TIMESTAMP_DIFF
from .stage import Stage


DELAY = 0.0001
class SimpleEstimator:
    def __init__(self, stage: Stage, integrator_type: type[StateIntegrator], maxlen=100, window = 50):
        self.input_stage = stage
        self._integrator = integrator_type()
        self._initial_state = State.default()
        self.buffer = deque(maxlen=maxlen) 
        self._running_ = True
        self.output_stage = Stage[State](
            sensors=[SampleType.STATE],
            maxlen=maxlen,
            window=window,
            stats=[TIMESTAMP_DIFF],
        )
        
        self.input_stage.subscribe(SampleType.IMU, lambda sample: self.buffer.append(sample))

    def stop(self):
        self._running_ = False

    def start(self):
        t = threading.Thread(target=self._estimator, daemon=False)
        t.start()

    def _estimator(self):
        state = self._initial_state
        last_timestamp = None
        while self._running_:
            
            if len(self.buffer) == 0:
                time.sleep(DELAY)  
                continue
            
            sample = self.buffer.popleft()
           
            if last_timestamp is None:
                # skip integration for the first sample
                last_timestamp = sample.timestamp
                continue
           
            dt = (sample.timestamp - last_timestamp) * 1e-9
            
            if dt > 1.0:
                last_timestamp = sample.timestamp
                continue
            
            print(f"Old state: {state.p} {state.v} {dt}")
            state = self._integrator.integrate(state, sample.gyro, sample.acc, dt)
            print(f"New state: {state.p} {state.v}") 
            
            self.output_stage.put(SampleType.STATE, state)
            
            last_timestamp = sample.timestamp
            
            
            
            
