#![no_std]
#![no_main]

use aux::{Direction, Led, Leds};
use core::convert::TryInto;
use core::f32::consts::PI;
use cortex_m::{asm, iprint, iprintln};
use cortex_m_rt::entry;
use lsm303agr::{Lsm303agr, Measurement, UnscaledMeasurement};
use micromath::F32Ext;
use panic_itm as _;
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

    loop {
        let data = sensor.mag_data_unscaled();

        if data.is_err() {
            // iprintln!(&mut itm.stim[0], "{}", ctr1);
            continue;
        }

        let UnscaledMeasurement { x, y, z } = data.unwrap();
        let x = x as f32 * 1.5;
        let y = y as f32 * 1.5;
        let z = z as f32 * 1.5;

        iprintln!(&mut itm.stim[0], "{}\t{}\t{}", x, y, z);
        // let theta = (y).atan2(x); // in radians

        // let dir = if theta < -7. * PI / 8. {
        //     Direction::North
        // } else if theta < -5. * PI / 8. {
        //     Direction::Northwest
        // } else if theta < -3. * PI / 8. {
        //     Direction::West
        // } else if theta < -PI / 8. {
        //     Direction::Southwest
        // } else if theta < PI / 8. {
        //     Direction::South
        // } else if theta < 3. * PI / 8. {
        //     Direction::Southeast
        // } else if theta < 5. * PI / 8. {
        //     Direction::East
        // } else if theta < 7. * PI / 8. {
        //     Direction::Northeast
        // } else {
        //     Direction::North
        // };

        // leds.iter_mut().for_each(|l| l.off());
        // leds[dir as usize].on();

        // let magnitude = (x * x + y * y + z * z).sqrt();

        // iprintln!(&mut itm.stim[0], "{:?}", magnitude);

        asm::delay(CYCLES_PER_MS * 100);
    }
}
