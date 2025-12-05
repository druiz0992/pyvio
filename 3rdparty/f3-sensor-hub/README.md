# F3-Sensor Hub
This project is an embedded Rust firmware for the **STM32F3 Discovery board**.
It continuously samples the on-board **accelerometer**, **gyroscope**, and **magnetometer** sensors and streams timestamped measurements over USB using either **ASCII** or **binary** encoding.
Additionally, it generates a 20 usec pulse in GPIO A0 every 10ms (48000 ticks) as a reference.

The firmware is intended for high-rate data logging, robotics, SLAM, and filter development (EKF, complementary filter, AHRS, etc.).


# Features

## ✔ High-rate IMU Sampling

* **Accelerometer:** 400 Hz
* **Gyroscope:** 400 Hz
* **Magnetometer:** 10 Hz
* All rates **configurable** at compile time.

## ✔ 48 MHz Timestamping

Samples are timestamped using a 32-bit cycle counter derived from the 48 MHz system clock, providing:

* Sub-microsecond resolution
* Monotonic timing
* No drift relative to sensor sampling tasks

## ✔ USB Streaming

The device appears to the host as a **USB CDC (virtual serial) device**.
All samples are pushed over USB as soon as they are collected.

You can choose between:

* **ASCII mode** — Human-readable CSV lines
* **Binary mode** — Compact and bandwidth-efficient 11-byte packets


# Sample Format

## Binary Format (11 bytes)

Each sample is represented as:

```
[0]     kind (u8)
[1..5]  timestamp (u32, little-endian)
[5..11] x, y, z components (3 × u16, little-endian)
```

Where:

| Sensor        | `kind`                      |
| ------------- | --------------------------- |
| Magnetometer  | `MAGNETOMETER` (e.g. 0x01)  |
| Accelerometer | `ACCELEROMETER` (e.g. 0x02) |
| Gyroscope     | `GYROSCOPE` (e.g. 0x03)     |

Rust implementation:

```rust
impl RawSample3D {
    pub fn to_bytes(&self) -> [u8; 11] {
        let mut buf = [0u8; 11];

        let (kind, ts, data) = match *self {
            RawSample3D::Magnetometer(ts, d) => (MAGNETOMETER, ts, d),
            RawSample3D::Accelerometer(ts, d) => (ACCELEROMETER, ts, d),
            RawSample3D::Gyroscope(ts, d) => (GYROSCOPE, ts, d),
        };

        buf[0] = kind;
        buf[1..5].copy_from_slice(&ts.to_le_bytes());
        buf[5..11].copy_from_slice(&helpers::u16x3_to_bytes(data));

        buf
    }
}
```


## ASCII Format

The ASCII representation produces a CSV string per sample:

```
A,12345678,-750,1023,15001
G,12345692,-5,2,0
M,12345710,120,32,900
```

Rust implementation:

```rust
pub fn to_ascii(&self) -> String<64> {
    let mut buf: String<64> = String::new();

    let (tag, ts, data) = match *self {
        RawSample3D::Magnetometer(ts, d) => ('M', ts, d),
        RawSample3D::Accelerometer(ts, d) => ('A', ts, d),
        RawSample3D::Gyroscope(ts, d) => ('G', ts, d),
    };

    write!(buf, "{},{}", tag, ts).ok();
    for x in data {
        write!(buf, ",{}", x).ok();
    }

    buf
}
```


## Building and Flashing

You need:

* `rustup`
* `thumbv7em-none-eabihf` target

Example:

```bash
rustup target add thumbv7em-none-eabihf
cargo build --release
```

# TODOs
- Clean code. Currently it is a mess. 
- Samples are currently written to a buffer during the ISR. I think this is likely the cause of high jitter (40-60 usec). Investigate alternative ways, such as writing samples in main loop provided I can guarantee that sample timestamp remains acurate, or maybe using DMA.
