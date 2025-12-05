#![no_main]
#![no_std]

mod globals;
mod helpers;
mod l3g4250d;
mod lsm303agr;
mod sample;

use defmt_rtt as _; // or defmt_itm
use panic_probe as _; // replaces panic-itm

use cortex_m::{
    asm::delay,
    interrupt::{self as irq, Mutex},
};
use cortex_m_rt::entry;

use stm32f3_discovery::leds::Leds;
use stm32f3xx_hal::{
    gpio::{self, gpioa::PA0, Alternate, Edge, OpenDrain, Output, PushPull},
    i2c::I2c,
    pac::{self, interrupt},
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
    time::rate::Hertz,
    usb::{Peripheral, UsbBus},
};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU16, AtomicU32, Ordering},
    u32,
};

use heapless::spsc::Queue;

use l3g4250d::L3g4250;
use lsm303agr::{Lsm303, ACCELEROMETER};
use sample::{RawSample3D, SampleStats};

type SclPin = gpio::Pin<gpio::Gpiob, gpio::U<PIN_6>, Alternate<OpenDrain, AF_4>>;
type SdaPin = gpio::Pin<gpio::Gpiob, gpio::U<PIN_7>, Alternate<OpenDrain, AF_4>>;
type I2cType = I2c<pac::I2C1, (SclPin, SdaPin)>;

type SckPin = gpio::Pin<gpio::Gpioa, gpio::U<PIN_5>, Alternate<PushPull, AF_5>>;
type MisoPin = gpio::Pin<gpio::Gpioa, gpio::U<PIN_6>, Alternate<PushPull, AF_5>>;
type MosiPin = gpio::Pin<gpio::Gpioa, gpio::U<PIN_7>, Alternate<PushPull, AF_5>>;
type SpiType = Spi<pac::SPI1, (SckPin, MisoPin, MosiPin)>;
type CsPin = gpio::Pin<gpio::Gpioe, gpio::U<PIN_3>, Output<PushPull>>;

const PIN_3: u8 = 3;
const PIN_5: u8 = 5;
const PIN_6: u8 = 6;
const PIN_7: u8 = 7;
const AF_4: u8 = 4;
const AF_5: u8 = 5;

const DECODE_FORMAT: &str = "BINARY";
const BURST_SIZE: usize = 10;
const PULSE_US: u16 = 70;

static LAST_TIMESTAMP_ACC: AtomicU32 = AtomicU32::new(0);
static LAST_TIMESTAMP_GYRO: AtomicU32 = AtomicU32::new(0);

static TIMER_SSN: AtomicU16 = AtomicU16::new(0);

static LSM303: Mutex<RefCell<Option<Lsm303<I2cType>>>> = Mutex::new(RefCell::new(None));
static L3G4250: Mutex<RefCell<Option<L3g4250<SpiType, CsPin>>>> = Mutex::new(RefCell::new(None));
static PPS_PIN: Mutex<RefCell<Option<PA0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

static mut SENSOR_QUEUE: Queue<RawSample3D, 128> = Queue::new();
static mut PENDING: Option<([u8; 11 * BURST_SIZE], usize)> = None;

fn init_tim2() {
    let rcc = unsafe { &*pac::RCC::ptr() };
    let tim2 = unsafe { &*pac::TIM2::ptr() };

    // 1️⃣ Enable TIM2 clock
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());

    // 2️⃣ Reset TIM2
    rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

    // 3️⃣ Stop timer
    tim2.cr1.modify(|_, w| w.cen().clear_bit());

    // 4️⃣ Full speed
    tim2.psc.write(|w| w.psc().bits(0));
    tim2.arr.write(|w| w.arr().bits(0xFFFF_FFFF));
    tim2.cnt.write(|w| w.cnt().bits(0));

    // 5️⃣ Enable timer
    tim2.cr1.modify(|_, w| w.cen().set_bit());
}

fn init_tim3() {
    let rcc = unsafe { &*pac::RCC::ptr() };
    let tim3 = unsafe { &*pac::TIM3::ptr() };

    // 1️⃣ Enable TIM3 clock
    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());

    // 2️⃣ Reset TIM3
    rcc.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());

    // 3️⃣ Stop timer
    tim3.cr1.modify(|_, w| w.cen().clear_bit());

    // 4️⃣ Set prescaler and auto-reload for 1 ms interrupt
    // PSC = 48_000 → 48 MHz / 48 = 1 MHz (1 us per tick)
    // ARR = 10000 → 10 ms period
    tim3.psc.write(|w| w.psc().bits(48 - 1));
    tim3.arr.write(|w| w.arr().bits(10000 - 1));

    // 5️⃣ Enable update interrupt
    tim3.dier.modify(|_, w| w.uie().set_bit());

    // 6️⃣ Enable counter
    tim3.cr1.modify(|_, w| w.cen().set_bit());

    // 7️⃣ Enable NVIC interrupt for TIM3
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM3) };
}

#[entry]
fn main() -> ! {
    // 1. Take ARM core peripherals (SysTick, NVIC)
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // 2. Take STM32 device peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // 3. Setup system clocks
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let freq = clocks.pclk1().0; // Hz
    let tick_time = 1.0 / freq as f32; // seconds per tick
    let tick_time_us = tick_time * 1_000_000.0; // µs per tick

    // 4. Split GPIO ports
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
    let mut gpiod = dp.GPIOD.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut gpiof = dp.GPIOF.split(&mut rcc.ahb);

    let pps = gpioa
        .pa0
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    irq::free(|cs| PPS_PIN.borrow(cs).replace(Some(pps)));

    // 5. Initialize board-wide components
    //    LEDs
    let mut leds = Leds::new(
        gpioe.pe8,
        gpioe.pe9,
        gpioe.pe10,
        gpioe.pe11,
        gpioe.pe12,
        gpioe.pe13,
        gpioe.pe14,
        gpioe.pe15,
        &mut gpioe.moder,
        &mut gpioe.otyper,
    );

    //    I2C
    let scl = gpiob
        .pb6
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let i2c = I2c::new(dp.I2C1, (scl, sda), Hertz(400_000), clocks, &mut rcc.apb1);

    // SPI pins
    let sck = gpioa
        .pa5
        .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi = gpioa
        .pa7
        .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    // CS pin
    let mut cs = gpioe
        .pe3
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    cs.set_high().unwrap(); // deselect gyro

    // SPI1
    let mut spi: Spi<pac::SPI1, _, u8> = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        Hertz(3_000_000),
        clocks,
        &mut rcc.apb2,
    );

    // USB
    let mut usb_dp = gpioa
        .pa12
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    usb_dp.set_low().ok();
    delay(clocks.sysclk().0 / 100);

    let usb_dm =
        gpioa
            .pa11
            .into_af14_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let usb_dp = usb_dp.into_af14_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    //   LSM303AGR
    let mut lsm303 = Lsm303::new(i2c);
    lsm303.init().unwrap_or_else(|_| {
        panic!("LSM303AGR init failed!");
    });
    irq::free(|cs| LSM303.borrow(cs).replace(Some(lsm303)));

    // L3g4250d gyro
    let mut l3g4250 = L3g4250::new(spi, cs);
    l3g4250.init().unwrap_or_else(|_| {
        panic!("L3G4250D init failed!");
    });
    irq::free(|cs| L3G4250.borrow(cs).replace(Some(l3g4250)));

    let mut drdy = gpioe
        .pe2
        .into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    drdy.make_interrupt_source(&mut syscfg);
    drdy.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    drdy.enable_interrupt(&mut dp.EXTI);

    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI2_TSC) };

    let mut int1 = gpioe
        .pe4
        .into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    int1.make_interrupt_source(&mut syscfg);
    int1.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    int1.enable_interrupt(&mut dp.EXTI);

    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI4) };

    let mut int2 = gpioe
        .pe1
        .into_pull_up_input(&mut gpioe.moder, &mut gpioe.pupdr);
    int2.make_interrupt_source(&mut syscfg);
    int2.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    int2.enable_interrupt(&mut dp.EXTI);

    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI1) };

    let mut sample_stats = SampleStats::default();
    let mut now = dp.TIM2.cnt.read().bits();
    let mut n_pkts = 0;

    init_tim2();
    init_tim3();
    defmt::info!("Starting application...");

    loop {
        usb_dev.poll(&mut [&mut serial]);
        if DECODE_FORMAT == "ASCII" {
            if let Some(sample) = unsafe { SENSOR_QUEUE.dequeue() } {
                defmt::println!("Dequeued: {:?}", sample);
                sample_stats.incr(&sample);
                let ascii_line = sample.to_ascii();
                let _ = serial.write(ascii_line.as_bytes());
                let _ = serial.write(b"\r\n");
            }
        } else {
            // -----------------------
            // Flush pending burst, if any
            // -----------------------
            unsafe {
                if let Some((ref mut buf, ref mut offset)) = PENDING {
                    match serial.write(&buf[*offset..11 * n_pkts]) {
                        Ok(n) if n > 0 => {
                            *offset += n;
                        }
                        _ => {
                            // Cannot write now, exit loop and retry in next iteration
                        }
                    }

                    // Finished sending -> clear pending burst
                    if *offset >= 11 * n_pkts {
                        PENDING = None;
                        n_pkts = 0;
                    }
                }
            }

            // -----------------------
            // If no pending burst, build next chunk
            // -----------------------
            unsafe {
                if PENDING.is_none() {
                    let mut burst = [0; 11 * BURST_SIZE];
                    while n_pkts < BURST_SIZE {
                        if let Some(sample) = SENSOR_QUEUE.dequeue() {
                            sample_stats.incr(&sample);
                            let sb = sample.to_bytes();
                            let start = n_pkts * sb.len();
                            let end = start + sb.len();
                            burst[start..end].copy_from_slice(&sb);
                            n_pkts += 1;
                        } else {
                            break;
                        }
                    }

                    if n_pkts > 0 {
                        PENDING = Some((burst, 0));
                    }
                }
            }
        }
        let curr = dp.TIM2.cnt.read().bits();
        if curr.wrapping_sub(now) > 5_000_000 {
            defmt::debug!("Samples: {}", sample_stats);
            now = curr;
        }
    }
}

#[interrupt]
fn EXTI2_TSC() {
    irq::free(|cs| {
        let dp = unsafe { pac::Peripherals::steal() };
        let now = dp.TIM2.cnt.read().bits();
        if let Some(sensor) = LSM303.borrow(cs).borrow_mut().as_mut() {
            let mut data = [0u8; 6];
            if sensor.read_magnetometer_data(&mut data).is_ok() {
                let sample = RawSample3D::Magnetometer(now, helpers::bytes6_to_u16x3(data));
                unsafe {
                    let _ = SENSOR_QUEUE.enqueue(sample);
                }
            }
        }
    });
    let exti = unsafe { &*pac::EXTI::ptr() };
    exti.pr1.write(|w| w.pr2().set_bit());
}

#[interrupt]
fn EXTI4() {
    irq::free(|cs| {
        let dp = unsafe { pac::Peripherals::steal() };
        let now = dp.TIM2.cnt.read().bits();
        let last_timestamp = LAST_TIMESTAMP_ACC.load(Ordering::Acquire);
        if let Some(sensor) = LSM303.borrow(cs).borrow_mut().as_mut() {
            let mut buf = [0u8; 1];
            let mut data = [0u8; 6];
            let n_samples = if sensor.read_register(ACCELEROMETER, 0x2F, &mut buf).is_ok() {
                (buf[0] & 0x1F) as u32
            } else {
                return;
            };

            LAST_TIMESTAMP_ACC.store(now, Ordering::Release);

            let incr = (now - last_timestamp) / n_samples;
            let mut timestamp = last_timestamp + incr;

            for _ in 0..n_samples {
                if sensor.read_accelerometer_data(&mut data).is_ok() {
                    let sample =
                        RawSample3D::Accelerometer(timestamp, helpers::bytes6_to_u16x3(data));
                    timestamp += incr;
                    unsafe {
                        let _ = SENSOR_QUEUE.enqueue(sample);
                    }
                }
            }
        }
    });

    let exti = unsafe { &*pac::EXTI::ptr() };
    exti.pr1.write(|w| w.pr4().set_bit());
}

#[interrupt]
fn EXTI1() {
    irq::free(|cs| {
        let dp = unsafe { pac::Peripherals::steal() };
        let now = dp.TIM2.cnt.read().bits();
        let last_timestamp = LAST_TIMESTAMP_GYRO.load(Ordering::Acquire);
        if let Some(sensor) = L3G4250.borrow(cs).borrow_mut().as_mut() {
            let mut buf = 0u8;
            let mut data = [0u8; 6];
            let n_samples = if sensor.read_register(0x2F, &mut buf).is_ok() {
                (buf & 0x1F) as u32
            } else {
                return;
            };

            LAST_TIMESTAMP_GYRO.store(now, Ordering::Release);

            let incr = (now - last_timestamp) / n_samples;
            let mut timestamp = last_timestamp + incr;

            for _ in 0..n_samples {
                if sensor.read_data(&mut data).is_ok() {
                    let sample = RawSample3D::Gyroscope(timestamp, helpers::bytes6_to_u16x3(data));
                    timestamp += incr;
                    unsafe {
                        let _ = SENSOR_QUEUE.enqueue(sample);
                    }
                }
            }
        }
    });

    let exti = unsafe { &*pac::EXTI::ptr() };
    exti.pr1.write(|w| w.pr1().set_bit());
}

#[interrupt]
fn TIM3() {
    irq::free(|cs| {
        let dp = unsafe { pac::Peripherals::steal() };
        let tim3 = dp.TIM3;

        let sr = tim3.sr.read();

        if sr.uif().bit_is_set() {
            let now = dp.TIM2.cnt.read().bits();
            let ssn = TIMER_SSN.load(Ordering::Acquire);
            let next = ssn.wrapping_add(1);
            TIMER_SSN.store(next, Ordering::Release);
            let sample = RawSample3D::Timer(now, [next, 0, 0]);
            unsafe {
                let _ = SENSOR_QUEUE.enqueue(sample);
            }
            tim3.sr.modify(|_, w| w.uif().clear_bit());

            if let Some(pps_pin) = PPS_PIN.borrow(cs).borrow_mut().as_mut() {
                let _ = pps_pin.set_high();
            }

            tim3.ccr1.write(|w| w.ccr().bits(PULSE_US));
        }

        if sr.cc1if().bit_is_set() {
            tim3.sr.modify(|_, w| w.cc1if().clear_bit());
            if let Some(pps_pin) = PPS_PIN.borrow(cs).borrow_mut().as_mut() {
                let _ = pps_pin.set_low();
            }
        }
    });
}
