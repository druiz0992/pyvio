use embedded_hal::blocking::i2c::{Write, WriteRead};

pub(crate) const MAGNETOMETER: u8 = 0b0011_110;

const WHO_AM_I_REG_M: u8 = 0x4f;
const CFG_REG_A_M: u8 = 0x60;
const CFG_REG_B_M: u8 = 0x61;
const CFG_REG_C_M: u8 = 0x62;
const OUT_X_L_M: u8 = 0x68;

const WHO_AM_I_VAL_M: u8 = 0b0100_0000;

#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum TemperatureCompensationMode {
    Disable = 0,
    #[default]
    Enable,
}

#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum OutputDataRate {
    #[default]
    Hz10 = 0,
    Hz20,
    Hz50,
    Hz100,
}

#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum SystemMode {
    #[default]
    ContinuousMode = 0,
    SingleMode = 1,
    IdleMode,
}

#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum DRDYMode {
    Disable = 0,
    #[default]
    Enable,
}

#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum LpfMode {
    Disable = 0,
    #[default]
    Enable,
}

#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum OffsetCancellationMode {
    Disable = 0,
    #[default]
    Enable,
}

#[derive(Debug, Default, Clone)]
pub(crate) struct Magnetometer {
    mode: SystemMode,
    rate: OutputDataRate,
    drdy_en: DRDYMode,
    temperature_compensation: TemperatureCompensationMode,
    lpf_en: LpfMode,
    offset_cancellation: OffsetCancellationMode,
}

impl Magnetometer {
    pub(crate) fn new() -> Self {
        Magnetometer::default()
    }

    fn set_mode(&mut self, mode: SystemMode) {
        self.mode = mode
    }

    fn set_temperature_compensation(&mut self, mode: TemperatureCompensationMode) {
        self.temperature_compensation = mode
    }

    fn sef_lpf(&mut self, mode: LpfMode) {
        self.lpf_en = mode
    }

    fn set_rate(&mut self, rate: OutputDataRate) {
        self.rate = rate
    }

    fn set_drdy(&mut self, ddry: DRDYMode) {
        self.drdy_en = ddry;
    }

    fn set_offset_cancellation(&mut self, mode: OffsetCancellationMode) {
        self.offset_cancellation = mode
    }

    fn get_mode(&self) -> SystemMode {
        self.mode.clone()
    }
    fn get_temperature_compensation(&self) -> TemperatureCompensationMode {
        self.temperature_compensation.clone()
    }

    fn get_rate(&self) -> OutputDataRate {
        self.rate.clone()
    }
    fn get_drdy(&self) -> DRDYMode {
        self.drdy_en.clone()
    }
    fn get_lpf(&self) -> LpfMode {
        self.lpf_en.clone()
    }
    fn get_offset_cancellation(&self) -> OffsetCancellationMode {
        self.offset_cancellation.clone()
    }

    pub(crate) fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let mode = self.get_mode();
        let rate = self.get_rate();
        let drdy = self.get_drdy();
        let temperature_comp = self.get_temperature_compensation();
        let lpf_en = self.get_lpf();
        let offset_cancellation = self.get_offset_cancellation();

        // Set mode and rate
        let val: u8 = mode as u8 | (rate as u8) << 2 | (temperature_comp as u8) << 7;
        i2c.write(MAGNETOMETER, &[CFG_REG_A_M, val])?;

        let val: u8 = (lpf_en as u8) << 0 | (offset_cancellation as u8) << 1;
        i2c.write(MAGNETOMETER, &[CFG_REG_B_M, val])?;

        // Configure DRDY mode
        let val = drdy as u8;
        i2c.write(MAGNETOMETER, &[CFG_REG_C_M, val])?;

        let mut buf = [0; 6];
        self.read_data(i2c, &mut buf)?;

        Ok(())
    }

    pub(crate) fn read_data<I2C, E>(&mut self, i2c: &mut I2C, buf: &mut [u8; 6]) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        i2c.write_read(MAGNETOMETER, &[OUT_X_L_M], buf)
    }
}
