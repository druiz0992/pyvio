from re import A
import yaml
from pathlib import Path
from typing import Any, Dict, Optional
from core.domain.samples import SensorType, SampleEncoding
from core.domain.sync_params import GPIO, SyncParams, parse_gpio


DEFAULT_SENSITIVITY = 1
DEFAULT_SAMPLE_WIDTH = 16
DEFAULT_CLOCK_HZ = 48_000_000
DEFAULT_BAUD_RATE = 115200
DEFAULT_PORT_NAME = "/dev/ttyACM1"
DEFAULT_ENCODING = SampleEncoding.BINARY
DEFAULT_SAMPLE_ANIMATION = False
DEFAULT_SYNC_GPIO: GPIO = GPIO.NONE


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
        self, sensor: Optional[SensorType] = None
    ) -> Dict[SensorType, float]:
        sensors = self._data.get("sensors", {})
        raw_sens = sensors.get("sensitivity", {})

        if sensor is None:
            return {stype: raw_sens[stype.name] for stype in SensorType}

        if sensor.name not in raw_sens:
            raise KeyError(f"Sensitivity not found for sensor {sensor.name}")
        return raw_sens[sensor.name]

    def sensor_sample_width(
        self, sensor: Optional[SensorType] = None
    ) -> Dict[SensorType, int]:
        sensors = self._data.get("sensors", {})
        raw_width = sensors.get("sample_width", {})

        if sensor is None:
            return {stype: raw_width[stype.name] for stype in SensorType}

        if sensor.name not in raw_width:
            raise KeyError(f"Sample wifth not found for sensor {sensor.name}")
        return raw_width[sensor.name]

    @property
    def clock_freq_hz(self):
        return self._data.get("clock_freq_hz", DEFAULT_CLOCK_HZ)

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
            return SyncParams(DEFAULT_SYNC_GPIO)

        return SyncParams(
            gpio=parse_gpio(imu_props.get("gpio", DEFAULT_SYNC_GPIO.value)),
        )

    def gps_sync(self) -> SyncParams:
        sync_props = self._data.get("sync", {})
        gps_props = sync_props.get("gps", {})

        if not gps_props:
            return SyncParams(DEFAULT_SYNC_GPIO)

        return SyncParams(
            gpio=parse_gpio(gps_props.get("gpio", DEFAULT_SYNC_GPIO.value)),
        )
