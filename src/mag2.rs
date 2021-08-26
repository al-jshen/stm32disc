#![no_std]
#![no_main]

use core::convert::TryInto;
use core::f32::consts::PI;
use cortex_m::{asm, iprint, iprintln};
use cortex_m_rt::entry;
use lsm303agr::{Lsm303agr, Measurement, UnscaledMeasurement};
use micromath::F32Ext;
use panic_itm as _;
use stm32disc::{Direction, Led, Leds};
use stm32f3xx_hal::{self as hal, pac, prelude::*};

const CYCLES_PER_SECOND: u32 = 8_000_000;
const CYCLES_PER_MS: u32 = 8_000;
const OCTANT: f32 = PI / 4.;

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
    let mut mag = match sensor.into_mag_continuous() {
        Ok(m) => m,
        Err(_) => panic!("fail"),
    };

    loop {
        iprintln!(&mut itm.stim[0], "{:?}", mag.mag_data());

        asm::delay(CYCLES_PER_MS * 10);
    }
}
