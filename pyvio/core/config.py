import yaml
from pathlib import Path
from typing import Any, Dict, Optional, List

from pyvio.core.domain.params.stream_params import StreamParams
from pyvio.core.ports.sample import SampleType

from .domain.sample import SampleEncoding
from ..core.domain.params.sync_params import GPIO, SyncParams, parse_gpio


DEFAULT_SENSITIVITY = 1
DEFAULT_SAMPLE_WIDTH = 16
DEFAULT_BAUD_RATE = 115200
DEFAULT_PORT_NAME = "/dev/ttyACM1"
DEFAULT_ENCODING = SampleEncoding.BINARY
DEFAULT_SAMPLE_ANIMATION = False
DEFAULT_SYNC_GPIO: GPIO = GPIO.NONE
DEFAULT_SYNC_FREQ_HZ = 0
DEFAULT_PHYPHOX_IP = "192.168.1.34"  
DEFAULT_PHYPHOX_SENSORS = SampleType.gps_list()


class Config:
    def __init__(self, path: str):
        """
        Load YAML configuration from the given path.

        Args:
            path: Path to config.yaml. Defaults to 'config/config.yaml' in project root.
        """
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(f"Config file not found: {self.path}")

        with open(self.path, "r") as f:
            self._data: Dict[str, Any] = yaml.safe_load(f)

    def get(self, key: str, default=None):
        """Get a config value by key."""
        return self._data.get(key, default)

    def sensor_sensitivity(
        self, sensor: Optional[SampleType] = None
    ) -> Dict[SampleType, float]:
        sensors = self._data.get("sensors", {})
        raw_sens = sensors.get("sensitivity", {})

        if sensor is None:
            return {stype: raw_sens[stype.name] for stype in SampleType if stype in SampleType.imu_list()}

        if sensor.name not in raw_sens:
            raise KeyError(f"Sensitivity not found for sensor {sensor.name}")
        return raw_sens[sensor.name]

    def sensor_sample_width(
        self, sensor: Optional[SampleType] = None
    ) -> Dict[SampleType, int]:
        sensors = self._data.get("sensors", {})
        raw_width = sensors.get("sample_width", {})

        if sensor is None:
            return {stype: raw_width[stype.name] for stype in SampleType if stype in SampleType.imu_list()}

        if sensor.name not in raw_width:
            raise KeyError(f"Sample wifth not found for sensor {sensor.name}")
        return raw_width[sensor.name]

    @property
    def sample_encoding(self) -> SampleEncoding:
        raw = self._data.get("sample_encoding", DEFAULT_ENCODING.to_str())
        try:
            return SampleEncoding[raw]
        except KeyError:
            return DEFAULT_ENCODING

    @property
    def sample_animation(self):
        return self._data.get("sample_animation", DEFAULT_SAMPLE_ANIMATION)

    @property
    def serial_port_name(self) -> str:
        serial_props = self._data.get("serial", {})
        return serial_props.get("port_name", DEFAULT_PORT_NAME)

    @property
    def serial_port_rate(self) -> int:
        serial_props = self._data.get("serial", {})
        return serial_props.get("baud_rate", DEFAULT_BAUD_RATE)

    def imu_sync(self) -> SyncParams:
        sync_props = self._data.get("sync", {})
        imu_props = sync_props.get("imu", {})

        if not imu_props:
            return SyncParams(DEFAULT_SYNC_GPIO, DEFAULT_SYNC_FREQ_HZ)

        return SyncParams(
            gpio=parse_gpio(imu_props.get("gpio", DEFAULT_SYNC_GPIO.value)),
            clock_freq_hz=imu_props.get("clock_freq_hz", DEFAULT_SYNC_FREQ_HZ)
        )

    def gps_sync(self) -> SyncParams:
        sync_props = self._data.get("sync", {})
        gps_props = sync_props.get("gps", {})

        if not gps_props:
            return SyncParams(DEFAULT_SYNC_GPIO, DEFAULT_SYNC_FREQ_HZ)

        return SyncParams(
            gpio=parse_gpio(gps_props.get("gpio", DEFAULT_SYNC_GPIO.value)),
            clock_freq_hz=gps_props.get("clock_freq_hz", DEFAULT_SYNC_FREQ_HZ)
        )
        
    def phyphox_ip(self) -> str:
        phyphox_props = self._data.get("phyphox", {})
        
        if not phyphox_props:
            return DEFAULT_PHYPHOX_IP
        
        return phyphox_props.get("ip", DEFAULT_PHYPHOX_IP)
        
    def phyphox_sensors(self) -> List[SampleType]:
        phyphox_props = self._data.get("phyphox", {})
        
        if not phyphox_props:
            return DEFAULT_PHYPHOX_SENSORS
        
        sensor_names = phyphox_props.get("sensors", [])
        if len(sensor_names):
            return DEFAULT_PHYPHOX_SENSORS
        
        sensors: List[SampleType] = []
        
        for name in sensor_names:
            try:
                new_sensor = SampleType[name]
            except KeyError:
                raise ValueError(f"Unknown sensor type: {name}")
            
            if new_sensor not in  SampleType.gps_list():
                 raise ValueError(f"Sensor '{name}' is not allowed for phyphox")
            
            sensors.append(new_sensor)
            
        return sensors
    
    def stream_params(self) -> StreamParams:
        stream_props = self._data.get("stream", {})
        return StreamParams.from_dict(stream_props)
        
           
        
