#![no_std]
#![no_main]

use aux::{Direction, Led, Leds};
use core::convert::TryInto;
use cortex_m::{asm, iprint, iprintln};
use cortex_m_rt::entry;
use lsm303agr::{Lsm303agr, Measurement};
use panic_itm as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

const CYCLES_PER_SECOND: u32 = 8_000_000;
const CYCLES_PER_MS: u32 = 8_000;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut leds = Leds::new(gpioe);

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
        .set_mag_odr(lsm303agr::MagOutputDataRate::Hz50)
        .unwrap();

    let mut ctr1 = 0;
    let mut ctr2 = 0;

    loop {
        let data = sensor.mag_data();

        if data.is_err() {
            ctr1 += 1;
            iprintln!(&mut itm.stim[0], "{}", ctr1);
            continue;
        }
        ctr2 += 1;

        iprintln!(&mut itm.stim[0], "{} - {:?}", ctr2, data);

        let Measurement { x, y, z } = data.unwrap();

        let dir = match (x > 0, y > 0) {
            (true, true) => Direction::Southeast,
            (true, false) => Direction::Southwest,
            (false, true) => Direction::Northeast,
            (false, false) => Direction::Northwest,
        };

        leds.iter_mut().for_each(|l| l.off());
        leds[dir as usize].on();

        asm::delay(CYCLES_PER_SECOND / 10);
    }
}
