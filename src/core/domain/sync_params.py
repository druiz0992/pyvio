from enum import Enum
from dataclasses import dataclass


class GPIO(Enum):
    GPIO4 = 4
    NONE = -1


DEFAULT_SYNC_GPIO = GPIO.NONE


def parse_gpio(value) -> GPIO:
    """
    Converts int, enum, or string into a GPIO enum.
    Accepts: GPIO(4), 4, "GPIO4", "none", "NONE", etc.
    """
    if isinstance(value, GPIO):
        return value

    if isinstance(value, int):
        # Match by integer value
        for g in GPIO:
            if g.value == value:
                return g
        raise ValueError(f"Invalid GPIO int: {value}")

    if isinstance(value, str):
        v = value.upper()
        if v in ("NONE", "GPIO_NONE"):
            return GPIO.NONE
        try:
            return GPIO[v]  # e.g. "GPIO4" → GPIO.GPIO4
        except KeyError:
            raise ValueError(f"Invalid GPIO name: '{value}'")

    raise TypeError(f"Cannot parse GPIO from value: {value!r}")


@dataclass
class SyncParams:
    gpio: GPIO

    @staticmethod
    def from_dict(imu_props: dict) -> "SyncParams":
        gpio_raw = imu_props.get("gpio", DEFAULT_SYNC_GPIO.value)
        gpio = parse_gpio(gpio_raw)
        return SyncParams(gpio)
