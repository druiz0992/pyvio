from core.domain.sync_params import SyncParams
from utils.kalman.simple import KalmanFilter
from collections import deque
from threading import Lock
from typing import Tuple


class Estimator:
    """Estimates drift and offset."""

    def __init__(self, window:int = 100, mcu_ppm:int = 50, host_jitter_us = 5):
        
        Q = (mcu_ppm * 1e-6)**2
        self._R = window * (host_jitter_us * 1e-6)**2
        self._kf = KalmanFilter(
            x0=[1.0],     # drift starts at ~1.0
            P0=[[1e-3]],  # initial uncerntainty. Quite confident drift is about 1.0
            A=[[1.0]],    # drift it constant. x[k] * A * x[k-1] + noise
            Q=[[Q]]       
        )
        
        self._buffer_mcu = deque(maxlen = window)
        self._buffer_host = deque(maxlen = window)
        self._buffer_host_abs = deque(maxlen = window)
        self._buffer_mcu_abs  = deque(maxlen = window)
        
        self._prev_mcu_t = 0
        self._prev_host_t = 0
        
        self._d_sum_mcu_t = 0
        self._d_sum_host_t = 0
        
        self._abs_sum_mcu_t = 0
        self._abs_sum_host_t = 0
        
        self._drift_offset = (0.0, 0.0)
        self._mcu_ssn = (0 ,0)
        
    def get_params(self) ->  Tuple[float, float]:
        # assigning tuples is atomic 
        return self._drift_offset
    
    def set_mcu(self, mcu_t: int, ssn: int):
        self._mcu_ssn = (mcu_t, ssn)

    def filter(self, host_t: float):
        mcu_t = self._mcu_ssn[0]
            
        if self._prev_host_t == 0 or self._prev_mcu_t == 0:
            self._prev_mcu_t = mcu_t
            self._prev_host_t = host_t
            return
        
        d_mcu_t = (mcu_t - self._prev_mcu_t) * 1e-9
        d_host_t = (host_t - self._prev_host_t) * 1e-9
        
        if len(self._buffer_mcu) == self._buffer_mcu.maxlen:
            self._d_sum_host_t -= self._buffer_host[0]
            self._d_sum_mcu_t -= self._buffer_mcu[0]
            self._abs_sum_host_t -= self._buffer_host_abs[0]
            self._abs_sum_mcu_t -= self._buffer_mcu_abs[0]
            
        self._buffer_host.append(d_host_t)
        self._buffer_mcu.append(d_mcu_t)
        
        host_t_sec = host_t * 1e-9
        mcu_t_sec = mcu_t * 1e-9
        self._buffer_host_abs.append(host_t_sec)
        self._buffer_mcu_abs.append(mcu_t_sec)
        
        self._d_sum_host_t += d_host_t
        self._d_sum_mcu_t += d_mcu_t
        
        self._abs_sum_host_t += host_t_sec
        self._abs_sum_mcu_t += mcu_t_sec
        
        if len(self._buffer_mcu) == self._buffer_mcu.maxlen:
            self._kf.predict()
            self._kf.update([self._d_sum_host_t], [[self._d_sum_mcu_t]], [[self._R]])
            drift = self._kf.x[0]
            mean_host = self._abs_sum_host_t / len(self._buffer_host_abs)
            mean_mcu  = self._abs_sum_mcu_t  / len(self._buffer_mcu_abs)
            offset = (mean_host - drift * mean_mcu) * 1e9
            self._drift_offset = (drift, offset)
            
        self._prev_mcu_t = mcu_t
        self._prev_host_t = host_t
