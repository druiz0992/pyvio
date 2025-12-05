mod accelerometer;
mod magnetometer;

use embedded_hal::blocking::i2c::{Write, WriteRead};

pub(crate) use accelerometer::{Accelerometer, ACCELEROMETER};
pub(crate) use magnetometer::{Magnetometer, MAGNETOMETER};

pub struct Lsm303<I2C> {
    i2c: I2C,
    magnetometer: Magnetometer,
    accelerometer: Accelerometer,
}

impl<I2C, E> Lsm303<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            accelerometer: Accelerometer::default(),
            magnetometer: Magnetometer::default(),
        }
    }

    pub fn init(&mut self) -> Result<(), E> {
        self.magnetometer.init(&mut self.i2c)?;
        self.accelerometer.init(&mut self.i2c)
    }

    pub fn read_register(&mut self, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(addr, &[reg], buf)?;

        Ok(())
    }

    pub fn write_register(&mut self, addr: u8, reg: u8) -> Result<(), E> {
        self.i2c.write(addr, &[reg])?;

        Ok(())
    }

    pub fn read_magnetometer_data(&mut self, buf: &mut [u8; 6]) -> Result<(), E> {
        self.magnetometer.read_data(&mut self.i2c, buf)
    }

    pub fn read_accelerometer_data(&mut self, buf: &mut [u8; 6]) -> Result<(), E> {
        self.accelerometer.read_data(&mut self.i2c, buf)
    }
}
