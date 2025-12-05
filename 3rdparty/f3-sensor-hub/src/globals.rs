use core::cell::RefCell;
use core::option::Option;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::ITM;

pub static ITM_GLOBAL: Mutex<RefCell<Option<&'static mut ITM>>> = Mutex::new(RefCell::new(None));
