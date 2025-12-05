import logging
import time

from pyvio.adapters.serial import SerialSensorAdapter
from pyvio.adapters.phyphox import PhyphoxSensorAdapter
from pyvio.core.ports.sensor import SensorPort
from pyvio.core.domain.pipeline.visualizer import LiveVisualizer
from pyvio.core.domain.pipeline.ahrs import Ahrs
from pyvio.core.domain.samples import SensorType
from pyvio.core.config import Config


logging.basicConfig(
    level=logging.ERROR,
    format="%(asctime)s [%(levelname)s] %(message)s",
)


def main():
    cfg = Config("./configs/config.yaml")
    imu: SensorPort = SerialSensorAdapter(cfg)
    phyphox: SensorPort = PhyphoxSensorAdapter(cfg)

    imu.start()
    phyphox.start()
    
    #visualizer = LiveVisualizer(phyphox.stage, SensorType.gps_list())
    ahrs = Ahrs(imu.stage, maxlen=100)

    ahrs.start_animation(interval=50)

    #visualizer.start_animation(interval=50)
    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
