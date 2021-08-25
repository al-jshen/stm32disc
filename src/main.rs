#![no_std]
#![no_main]

use core::convert::TryInto;
use cortex_m::{asm, iprint, iprintln};
use cortex_m_rt::entry;
use lsm303agr::Lsm303agr;
use panic_itm as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

const CYCLES_PER_SECOND: u32 = 8_000_000;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let mut scl =
        gpiob
            .pb6
            .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut sda =
        gpiob
            .pb7
            .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    scl.internal_pull_up(&mut gpiob.pupdr, true);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = hal::i2c::I2c::new(
        dp.I2C1,
        (scl, sda),
        400.kHz().try_into().unwrap(),
        clocks,
        &mut rcc.apb1,
    );

    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    sensor.init().unwrap();
    sensor
        .set_mag_odr(lsm303agr::MagOutputDataRate::Hz100)
        .unwrap();

    loop {
        match sensor.mag_data() {
            Ok(val) => iprintln!(&mut itm.stim[0], "{:?}", val),
            Err(_) => asm::delay(CYCLES_PER_SECOND),
        }
    }
}
