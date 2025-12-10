from pyvio.core.domain.pipeline.consumer import Consumer
from pyvio.core.config import Config
from pyvio.core.domain.pipeline.visualizer import LiveVisualizer
from pyvio.core.domain.pipeline.visualizer_state2d import LiveVisualizerState2D
from pyvio.core.domain.pipeline.visualizer_state3d import LiveVisualizerState3D
from pyvio.core.domain.sample import SensorSample
from pyvio.core.domain.state import State
from pyvio.core.domain.pipeline.ahrs import Ahrs


def main():
    cfg = Config("./configs/receiver.yaml")

    consumer = Consumer(cfg.stream_params(), State)
    consumer.start()

    visualizer = LiveVisualizerState3D(consumer.stage)
    # ahrs = Ahrs(consumer.stage, maxlen=100)

    # ahrs.start_animation(interval=50)

    visualizer.start(interval=50)


if __name__ == "__main__":
    main()
