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
    sensor.set_accel_scale(lsm303agr::AccelScale::G16).unwrap();
    sensor.set_accel_mode(lsm303agr::AccelMode::Normal).unwrap();
    sensor
        .set_accel_odr(lsm303agr::AccelOutputDataRate::Khz1_344)
        .unwrap();

    let mut max_accel = 0;
    let mut recording = false;
    let mut ctr = 0;

    loop {
        let data = sensor.accel_data();

        if data.is_err() {
            continue;
        }

        let Measurement { x, .. } = data.unwrap();
        let x = x.abs();

        // above threshold, start recording
        if x > 1000 {
            if recording {
                max_accel = x.max(max_accel);
                ctr += 1;
            } else {
                recording = true;
                max_accel = x;
            }
        } else {
            // reset
            if recording {
                iprintln!(
                    &mut itm.stim[0],
                    "{} g's across {} ticks",
                    max_accel as f32 / 1000.,
                    ctr
                );
                max_accel = 0;
                ctr = 0;
                recording = false;
            }
        }

        asm::delay(CYCLES_PER_MS / 2);
    }
}
