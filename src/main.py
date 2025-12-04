import logging
import time

from adapters.serial import SerialSensorAdapter
from adapters.phyphox import PhyphoxSensorAdapter
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
    imu: SensorPort = SerialSensorAdapter(cfg)
    phyphox: SensorPort = PhyphoxSensorAdapter(cfg)

    imu.start()
    phyphox.start()
    
    #visualizer = LiveVisualizer(phyphox.stage, SensorType.gps_list())
    #ahrs = Ahrs(sensor_adapter.stage, maxlen=100)

    #ahrs.start_animation(interval=50)

    #visualizer.start_animation(interval=50)
    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
