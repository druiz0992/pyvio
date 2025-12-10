import logging
import time

from pyvio.adapters.mock_sensor import MockIMUSensor, MockGPSSensor
from pyvio.core.ports.sample import SampleType
from pyvio.core.domain.pipeline.eskf import ESKFStage
from pyvio.core.domain.pipeline.interpolator import Interpolator
from pyvio.core.domain.pipeline.eskf import ESKFStage
from pyvio.core.services.interpolator import LinearInterpolator
from pyvio.utils.trajectory import SpiralTrajectory


logging.basicConfig(
    level=logging.ERROR,
    format="%(asctime)s [%(levelname)s] %(message)s",
)


def main():
    trajectory = SpiralTrajectory(radius=5.0, height = 5.0, speed = 1.0)
    imu = MockIMUSensor(trajectory, dt= 1/400)
    gps = MockGPSSensor(trajectory, dt = 1)
    
    #interpolator = Interpolator(imu.stage, SampleType.interp_list(), LinearInterpolator)
    #estimator = ESKFStage([interpolator.output_stage, gps.stage])
    
    imu.start()
    gps.start()
    #interpolator.start()
    #estimator.start()
    
    # ahrs = Ahrs(imu.stage, maxlen=100)

    # ahrs.start(interval=50)

    # visualizer.start(interval=50)
    while True:
        time.sleep(0.1)


if __name__ == "__main__":
    main()
