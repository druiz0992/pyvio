use crate::helpers;
use core::fmt::Write;
use defmt::Format;
use heapless::String;

const MAGNETOMETER: u8 = 0;
const ACCELEROMETER: u8 = 1;
const GYROSCOPE: u8 = 2;
const TIMER: u8 = 3;

#[derive(Debug, Clone, Copy, Format)]
pub enum RawSample3D {
    Magnetometer(u32, [u16; 3]),
    Accelerometer(u32, [u16; 3]),
    Gyroscope(u32, [u16; 3]),
    Timer(u32, [u16; 3]),
}

impl RawSample3D {
    pub fn to_bytes(&self) -> [u8; 11] {
        let mut buf = [0u8; 11];

        let (kind, ts, data) = match *self {
            RawSample3D::Magnetometer(ts, d) => (MAGNETOMETER, ts, d),
            RawSample3D::Accelerometer(ts, d) => (ACCELEROMETER, ts, d),
            RawSample3D::Gyroscope(ts, d) => (GYROSCOPE, ts, d),
            RawSample3D::Timer(ts, d) => (TIMER, ts, d),
        };

        buf[0] = kind;
        buf[1..5].copy_from_slice(&ts.to_le_bytes());
        buf[5..11].copy_from_slice(&helpers::u16x3_to_bytes(data));

        buf
    }

    pub fn to_ascii(&self) -> String<64> {
        let mut buf: String<64> = String::new();

        let (tag, ts, data) = match *self {
            RawSample3D::Magnetometer(ts, d) => ('M', ts, d),
            RawSample3D::Accelerometer(ts, d) => ('A', ts, d),
            RawSample3D::Gyroscope(ts, d) => ('G', ts, d),
            RawSample3D::Timer(ts, d) => ('T', ts, d),
        };

        write!(buf, "{},{}", tag, ts).ok();
        for x in data {
            write!(buf, ",{}", x).ok();
        }

        buf
    }
}

#[derive(Debug, Format, Default)]
pub(crate) struct SampleStats {
    n_magnetometer: u16,
    n_accelerometer: u16,
    n_gyroscope: u16,
    n_timer: u16,
}

impl SampleStats {
    pub fn incr(&mut self, sample: &RawSample3D) {
        match sample {
            RawSample3D::Accelerometer(_, _) => self.n_accelerometer += 1,
            RawSample3D::Magnetometer(_, _) => self.n_magnetometer += 1,
            RawSample3D::Gyroscope(_, _) => self.n_gyroscope += 1,
            RawSample3D::Timer(_, _) => self.n_timer += 1,
        }
    }
}
