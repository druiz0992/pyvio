import logging
import time

from pyvio.adapters.serial import SerialSensorAdapter
from pyvio.adapters.phyphox import PhyphoxSensorAdapter
from pyvio.core.domain.pipeline.simple_estimator import SimpleEstimator
from pyvio.core.ports.sensor import SensorPort
from pyvio.core.domain.pipeline.visualizer import LiveVisualizer
from pyvio.core.domain.pipeline.ahrs import Ahrs
from pyvio.core.domain.pipeline.streamer import Streamer
from pyvio.core.ports.sample import SampleType
from pyvio.core.domain.pipeline.interpolator import Interpolator
from pyvio.core.domain.interpolator import LinearInterpolator, MidpointInterpolator
from pyvio.core.domain.integrator import EulerIntegrator, RK4Integrator
from pyvio.core.config import Config


logging.basicConfig(
    level=logging.ERROR,
    format="%(asctime)s [%(levelname)s] %(message)s",
)


def main():
    cfg = Config("./configs/config.yaml")
    imu: SensorPort = SerialSensorAdapter(cfg)

    interpolator = Interpolator(imu.stage, SampleType.interp_list(), LinearInterpolator)
    estimator = SimpleEstimator(interpolator.output_stage, EulerIntegrator)
    # phyphox: SensorPort = PhyphoxSensorAdapter(cfg)

    streamer = Streamer([estimator.output_stage], cfg.stream_params())

    imu.start()
    interpolator.start()
    estimator.start()
    streamer.start()
    # phyphox.start()

    # visualizer = LiveVisualizer(phyphox.stage, SampleType.gps_list())
    # ahrs = Ahrs(imu.stage, maxlen=100)

    # ahrs.start(interval=50)

    # visualizer.start(interval=50)
    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
