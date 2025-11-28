import logging

from adapters.serial import SerialSensorAdapter
from core.ports.sensor import SensorPort
from core.domain.pipeline.visualizer import LiveVisualizer
from core.domain.pipeline.ahrs import Ahrs
from core.domain.samples import SensorType
from core.config import Config


logging.basicConfig(
    level=logging.ERROR,
    format="%(asctime)s [%(levelname)s] %(message)s",
)


def main():
    cfg = Config("./config.yaml")
    sensors = SensorType.list()
    sensor_adapter: SensorPort = SerialSensorAdapter(sensors, cfg)

    # visualizer = LiveVisualizer(sensor_adapter.stage, maxlen=1000)
    ahrs = Ahrs(sensor_adapter.stage, maxlen=100)

    sensor_adapter.start()
    ahrs.start_animation(interval=50)

    # visualizer.start_animation(interval=50)


if __name__ == "__main__":
    main()
