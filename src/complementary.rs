#![no_std]
#![no_main]

use core::convert::TryInto;
use cortex_m::iprintln;
use cortex_m_rt::entry;
use hal::{
    delay::Delay,
    spi::{config::Config, Spi},
};
use l3gd20::{I16x3, L3gd20, Odr};
use lsm303agr::{Lsm303agr, Measurement};
use micromath::F32Ext;
use panic_itm as _;
// use stm32disc::{Direction, Led, Leds};
use stm32f3xx_hal::{self as hal, pac, prelude::*};

const TS_MS: u32 = 1000 / 95;
const TS_S: f32 = TS_MS as f32 / 1000.;
const ALPHA: f32 = 0.98;
const RAD_TO_DEG: f32 = 180. / core::f32::consts::PI;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;
    let syst = cp.SYST;

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(syst, clocks);

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
    sensor.set_accel_scale(lsm303agr::AccelScale::G2).unwrap();
    sensor
        .set_accel_mode(lsm303agr::AccelMode::HighResolution)
        .unwrap();
    sensor
        .set_accel_odr(lsm303agr::AccelOutputDataRate::Hz100)
        .unwrap();

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut ncs = gpioe
        .pe3
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    ncs.set_high().unwrap();

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut sck =
        gpioa
            .pa5
            .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mut miso =
        gpioa
            .pa6
            .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mut mosi =
        gpioa
            .pa7
            .into_af5_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    sck.internal_pull_up(&mut gpioa.pupdr, true);
    miso.internal_pull_up(&mut gpioa.pupdr, true);
    mosi.internal_pull_up(&mut gpioa.pupdr, true);

    let spi = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        Config::default(),
        clocks,
        &mut rcc.apb2,
    );

    let mut gyro = L3gd20::new(spi, ncs).unwrap();
    gyro.set_scale(l3gd20::Scale::Dps250).unwrap();
    gyro.set_odr(Odr::Hz95).unwrap();

    let sens = gyro.scale().unwrap();

    // gyro calibration

    let mut gyro_bias_x = 0.;
    let mut gyro_bias_y = 0.;

    // welford mean
    for i in 1..501 {
        let I16x3 { x: gx, y: gy, .. } = gyro.gyro().unwrap();
        let delta_x = gx as f32 - gyro_bias_x;
        gyro_bias_x += delta_x / i as f32;
        let delta_y = gy as f32 - gyro_bias_y;
        gyro_bias_y += delta_y / i as f32;
    }

    let gyro_bias_x = gyro_bias_x as i16;
    let gyro_bias_y = gyro_bias_y as i16;

    let mut roll = 0.;
    let mut pitch = 0.;

    loop {
        let accel = sensor.accel_data();

        if accel.is_err() {
            continue;
        }

        let Measurement {
            x: ax,
            y: ay,
            z: az,
        } = accel.unwrap();

        let I16x3 { x: gx, y: gy, .. } = gyro.gyro().unwrap();

        // complementary filter
        let accel_roll = (ay as f32).atan2(az as f32) * RAD_TO_DEG;
        let gyro_roll = sens.degrees(gx - gyro_bias_x) * TS_S + roll;
        roll = ALPHA * gyro_roll + (1. - ALPHA) * accel_roll;

        let accel_pitch = (ax as f32).atan2(az as f32) * RAD_TO_DEG;
        let gyro_pitch = sens.degrees(gy - gyro_bias_y) * TS_S + pitch;
        pitch = ALPHA * gyro_pitch + (1. - ALPHA) * accel_pitch;

        iprintln!(&mut itm.stim[0], "{} {}", roll, pitch);

        delay.delay_ms(TS_MS);
    }
}
