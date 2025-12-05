use embedded_hal::blocking::i2c::{Write, WriteRead};

pub(crate) const ACCELEROMETER: u8 = 0b0011_001;
const WHO_AM_I_REG_M: u8 = 0x0F;
const CTRL_REG1_A: u8 = 0x20;
const CTRL_REG3_A: u8 = 0x22;
const CTRL_REG4_A: u8 = 0x23;
const CTRL_REG5_A: u8 = 0x24;
const OUT_X_L_A: u8 = 0x28;
const FIFO_CTRL_REG_A: u8 = 0x2E;
const FIFO_SRC_REG_A: u8 = 0x2F;

const WHO_AM_I_VAL_A: u8 = 0b0011_0011;

// CTRL_REG1_A
const LP_EN_OFFSET: u8 = 3;
const ODR_OFFSET: u8 = 4;
const XYZ_EN_OFFSET: u8 = 0;
const XYZ_EN: u8 = 7;
// CTRL_REG3_A
const I1_WTM_OFFSET: u8 = 2;
// CTRL_REG4_A
const HR_OFFSET: u8 = 3;
const FS_OFFSET: u8 = 4;
// CTRL_REG5_A
const FIFO_EN_OFFSET: u8 = 6;
// FIFO_CTRL_REG_A
const FTH_DEFAULT_VALUE: u8 = 16;
const FTH_OFFSET: u8 = 0;
const TR_OFFSET: u8 = 5;
const FM_OFFSET: u8 = 6;
// FIFO_SRC_REG_A
const FSS_OFFSET: u8 = 0;
const FSS_MASK: u8 = 0x1F;
const EMPTY_OFFSET: u8 = 5;

// CTRL_REG1_A
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum OutputDataRate {
    PowerDown = 0,
    Hz1,
    Hz10,
    Hz25,
    Hz50,
    Hz100,
    Hz200,
    #[default]
    Hz400,
    Hz1_620,
    Hz1_344,
}

// CTRL_REG1_A
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum PowerMode {
    #[default]
    Normal = 0,
    Low,
}

// CTRL_REG3_A
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum WatermarkMode {
    Disable = 0,
    #[default]
    Enable,
}

// CTRL_REG4_A  (HR)
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum ResolutionMode {
    Low = 0,
    #[default]
    High,
}

// CTRL_REG4_A (FS)
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum GScaleMode {
    #[default]
    G2 = 0,
    G4,
    G8,
    G16,
}

// CTRL_REG5_A
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum FifoEnable {
    Disable = 0,
    #[default]
    Enable,
}

// FIFO_CTRL_REG_A
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum TriggerMode {
    #[default]
    INT1 = 0,
    INT2,
}

// FIFO_CTRL_REG_A
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum FifoMode {
    Bypass = 0,
    Fifo,
    #[default]
    Stream,
    StreamToFifo,
}

#[derive(Debug, Default, Clone)]
struct FifoConfig {
    mode: FifoMode,
    enable: FifoEnable,
    watermark: WatermarkMode,
    trigger: TriggerMode,
}

#[derive(Debug, Default, Clone)]
pub(crate) struct Accelerometer {
    fifo: FifoConfig,
    g_scale: GScaleMode,
    resolution: ResolutionMode,
    rate: OutputDataRate,
    power: PowerMode,
}

impl Accelerometer {
    pub(crate) fn new() -> Self {
        Accelerometer::default()
    }

    fn set_fifo_enable(&mut self, enable: FifoEnable) {
        self.fifo.enable = enable;
    }

    fn set_fifo(&mut self, mode: FifoMode, watermark: WatermarkMode, trigger: TriggerMode) {
        self.fifo.mode = mode;
        self.fifo.watermark = watermark;
        self.fifo.trigger = trigger;
    }

    fn set_g_scale(&mut self, scale: GScaleMode) {
        self.g_scale = scale;
    }

    fn set_resolution(&mut self, resolution: ResolutionMode) {
        self.resolution = resolution;
    }

    fn set_rate(&mut self, rate: OutputDataRate) {
        self.rate = rate;
    }

    fn set_power_mode(&mut self, mode: PowerMode) {
        self.power = mode;
    }

    fn get_fifo_config(&self) -> FifoConfig {
        self.fifo.clone()
    }

    fn get_g_scale(&self) -> GScaleMode {
        self.g_scale.clone()
    }

    fn get_resolution(&self) -> ResolutionMode {
        self.resolution.clone()
    }

    fn get_rate(&self) -> OutputDataRate {
        self.rate.clone()
    }

    fn get_power_mode(&self) -> PowerMode {
        self.power.clone()
    }

    pub(crate) fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let fifo_config = self.get_fifo_config();
        let g_scale = self.get_g_scale();
        let resolution = self.get_resolution();
        let rate = self.get_rate();
        let power = self.get_power_mode();

        // Set low power mode and rate
        let val: u8 =
            (power as u8) << LP_EN_OFFSET | (rate as u8) << ODR_OFFSET | (XYZ_EN << XYZ_EN_OFFSET);
        i2c.write(ACCELEROMETER, &[CTRL_REG1_A, val])?;

        // Configure interrupt watermark mode
        let val = (fifo_config.watermark as u8) << I1_WTM_OFFSET;
        i2c.write(ACCELEROMETER, &[CTRL_REG3_A, val])?;

        // Resolution and G Scale
        let val = (resolution as u8) << HR_OFFSET | (g_scale as u8) << FS_OFFSET;
        i2c.write(ACCELEROMETER, &[CTRL_REG4_A, val])?;

        // FIFO enable
        let val = (fifo_config.enable as u8) << FIFO_EN_OFFSET;
        i2c.write(ACCELEROMETER, &[CTRL_REG5_A, val])?;

        let mut val = (FTH_DEFAULT_VALUE << FTH_OFFSET) | (fifo_config.trigger as u8) << TR_OFFSET;
        i2c.write(ACCELEROMETER, &[FIFO_CTRL_REG_A, val])?;
        val |= (fifo_config.mode as u8) << FM_OFFSET;
        i2c.write(ACCELEROMETER, &[FIFO_CTRL_REG_A, val])?;

        let mut buf = [0u8];
        i2c.write_read(ACCELEROMETER, &[0x31], &mut buf)?;

        Ok(())
    }

    pub(crate) fn read_data<I2C, E>(&mut self, i2c: &mut I2C, buf: &mut [u8; 6]) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        i2c.write_read(ACCELEROMETER, &[OUT_X_L_A | 0x80], buf)
    }
}
