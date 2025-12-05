from pyvio.core.domain.pipeline.consumer import Consumer
from pyvio.core.config import Config
from pyvio.core.domain.pipeline.visualizer import LiveVisualizer
from pyvio.core.domain.samples import SensorType
from pyvio.core.domain.pipeline.ahrs import Ahrs


def main():
    cfg = Config("./configs/receiver.yaml")

    consumer = Consumer(cfg.stream_params())
    consumer.start()

    # visualizer = LiveVisualizer(consumer.stage, SensorType.list())
    ahrs = Ahrs(consumer.stage, maxlen=100)

    ahrs.start_animation(interval=50)

    # visualizer.start_animation(interval=50)


if __name__ == "__main__":
    main()
