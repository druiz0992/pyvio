use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

const WHO_AM_I_REG_M: u8 = 0xF;
const CTRL_REG1_G: u8 = 0x20;
const CTRL_REG3_G: u8 = 0x22;
const CTRL_REG4_G: u8 = 0x23;
const CTRL_REG5_G: u8 = 0x24;
const OUT_X_L_G: u8 = 0x28;
const FIFO_CTRL_REG_G: u8 = 0x2E;
const FIFO_SRC_REG_G: u8 = 0x2F;

// CTRL_REG1_G
const ODR_OFFSET: u8 = 6;
const BW_OFFSET: u8 = 4;
const PD_OFFSET: u8 = 3;
const XYZ_EN_OFFSET: u8 = 0;
const XYZ_EN_VAL: u8 = 7;

// CTRL_REG3_G
const I2_WTM_OFFSET: u8 = 2;
// CTRL_REG4_G
const SIM_OFFSET: u8 = 0;
const FS_OFFSET: u8 = 4;
// CTRL_REG5_G
const FIFO_EN_OFFSET: u8 = 6;
// FIFO_CTRL_REG_G
const FTH_DEFAULT_VALUE: u8 = 16;
const FTH_OFFSET: u8 = 0;
const FM_OFFSET: u8 = 5;
// FIFO_SRC_REG_G
const FSS_OFFSET: u8 = 0;
const FSS_MASK: u8 = 0x1F;
const EMPTY_OFFSET: u8 = 5;

// CTRL_REG1_G
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum OutputDataRate {
    Hz100 = 0,
    Hz200,
    #[default]
    Hz400,
    Hz800,
}

// CTRL_REG1_G
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum BWMode {
    Cutoff0 = 0,
    Cutoff1,
    #[default]
    Cutoff2,
    Cutoff3,
}

// CTRL_REG1_G
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum PowerMode {
    PowerDown = 0,
    #[default]
    Normal,
}

// CTRL_REG3_G
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum WatermarkMode {
    Disable = 0,
    #[default]
    Enable,
}

// CTRL_REG4_G  (SIM)
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum SimMode {
    #[default]
    SPI4Wire = 0,
    SPI3Wire,
}

// CTRL_REG4_G (FS)
// degrees per second
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum ScaleMode {
    #[default]
    DPS245 = 0,
    DPS500,
    DPS2000,
}

// CTRL_REG5_G
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum FifoEnable {
    Disable = 0,
    #[default]
    Enable,
}

// FIFO_CTRL_REG_G
#[derive(Debug, Default, Clone)]
#[repr(u8)]
enum FifoMode {
    Bypass = 0,
    Fifo,
    #[default]
    Stream,
}

#[derive(Debug, Default, Clone)]
struct FifoConfig {
    mode: FifoMode,
    enable: FifoEnable,
    watermark: WatermarkMode,
}

#[derive(Debug, Default, Clone)]
pub(crate) struct L3g4250<SPI, CS> {
    fifo: FifoConfig,
    scale: ScaleMode,
    rate: OutputDataRate,
    bw: BWMode,
    sim: SimMode,
    power: PowerMode,
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E> L3g4250<SPI, CS>
where
    SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        L3g4250 {
            spi,
            cs,
            fifo: FifoConfig::default(),
            scale: ScaleMode::default(),
            rate: OutputDataRate::default(),
            bw: BWMode::default(),
            sim: SimMode::default(),
            power: PowerMode::default(),
        }
    }
    fn set_fifo_enable(&mut self, enable: FifoEnable) {
        self.fifo.enable = enable;
    }

    fn set_fifo_config(&mut self, mode: FifoMode, watermark: WatermarkMode) {
        self.fifo.mode = mode;
        self.fifo.watermark = watermark;
    }

    fn set_scale(&mut self, scale: ScaleMode) {
        self.scale = scale;
    }

    fn set_rate(&mut self, rate: OutputDataRate) {
        self.rate = rate;
    }

    fn set_power_mode(&mut self, mode: PowerMode) {
        self.power = mode;
    }

    fn set_bw_mode(&mut self, mode: BWMode) {
        self.bw = mode;
    }

    fn set_sim_mode(&mut self, mode: SimMode) {
        self.sim = mode;
    }

    fn get_fifo_config(&self) -> FifoConfig {
        self.fifo.clone()
    }

    fn get_scale(&self) -> ScaleMode {
        self.scale.clone()
    }

    fn get_rate(&self) -> OutputDataRate {
        self.rate.clone()
    }

    fn get_power_mode(&self) -> PowerMode {
        self.power.clone()
    }

    fn get_bw_mode(&self) -> BWMode {
        self.bw.clone()
    }

    fn get_sim_mode(&self) -> SimMode {
        self.sim.clone()
    }

    pub fn init(&mut self) -> Result<(), E> {
        let fifo_config = self.get_fifo_config();
        let scale = self.get_scale();
        let rate = self.get_rate();
        let power = self.get_power_mode();
        let bw = self.get_bw_mode();
        let sim = self.get_sim_mode();

        // Set low power mode and rate
        let val: u8 = (power as u8) << PD_OFFSET
            | (rate as u8) << ODR_OFFSET
            | (bw as u8) << BW_OFFSET
            | (XYZ_EN_VAL << XYZ_EN_OFFSET);
        self.write_register(CTRL_REG1_G, val)?;

        // Configure interrupt watermark mode
        let val = (fifo_config.watermark as u8) << I2_WTM_OFFSET;
        self.write_register(CTRL_REG3_G, val)?;

        // Resolution and SPI wires
        let val = (scale as u8) << FS_OFFSET | (sim as u8) << SIM_OFFSET;
        self.write_register(CTRL_REG4_G, val)?;

        // FIFO enable
        let val = (fifo_config.enable as u8) << FIFO_EN_OFFSET;
        self.write_register(CTRL_REG5_G, val)?;

        let val = (FTH_DEFAULT_VALUE << FTH_OFFSET) | (fifo_config.mode as u8) << FM_OFFSET;
        self.write_register(FIFO_CTRL_REG_G, val)?;

        self.flush_fifo()?;

        Ok(())
    }

    fn flush_fifo(&mut self) -> Result<(), E> {
        let mut buf = 0u8;
        if let Ok(_) = self.read_register(0x2F, &mut buf) {
            let n_samples = (buf & 0x1F) as u32;
            let mut data = [0u8; 6];
            for _ in 0..n_samples {
                self.read_data(&mut data)?;
            }
        }
        Ok(())
    }

    pub fn write_register(&mut self, reg: u8, val: u8) -> Result<(), E> {
        let addr = reg & 0x7F; // MSB=0 → write
        self.cs.set_low().ok();
        let r = self.spi.write(&[addr, val]);
        self.cs.set_high().ok();
        r
    }

    pub fn read_register(&mut self, reg: u8, buf: &mut u8) -> Result<(), E> {
        // Single-byte read: set bit7, no auto-increment
        let addr = reg | 0x80;

        let mut tx = [addr, 0x00]; // send addr, receive one byte
        self.cs.set_low().ok();
        let result = self.spi.transfer(&mut tx);
        self.cs.set_high().ok();

        match result {
            Ok(rx) => {
                *buf = rx[1]; // second byte is the data
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    pub fn read_data(&mut self, buffer: &mut [u8]) -> Result<(), E> {
        // Read + auto-increment
        let addr = OUT_X_L_G | 0xC0;

        self.cs.set_low().ok();
        self.spi.write(&[addr])?;
        self.spi.transfer(buffer)?;
        self.cs.set_high().ok();

        Ok(())
    }
}
