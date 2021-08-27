#![no_std]
#![no_main]

use core::convert::TryInto;
use cortex_m::iprintln;
use cortex_m_rt::entry;
use hal::{
    delay::Delay,
    pac::TIM2,
    spi::{config::Config, Spi},
};
use l3gd20::{I16x3, L3gd20, Odr};
use lsm303agr::{Lsm303agr, Measurement};
use micromath::F32Ext;
use panic_itm as _;
// use stm32disc::{Direction, Led, Leds};
use libm::atan2f;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

const TS_US: u32 = 20000;
const TS_MS: f32 = TS_US as f32 / 1000.;
const TS_S: f32 = TS_MS as f32 / 1000.;
const CLOCKSPD: u32 = 8_000_000;
const ALPHA: f32 = 0.98;
const RAD_TO_DEG: f32 = 180. / core::f32::consts::PI;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;
    let syst = cp.SYST;

    let dp = pac::Peripherals::take().unwrap();

    dp.RCC.apb1enr.write(|w| w.tim2en().set_bit());

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let tim = unsafe { &*TIM2::ptr() };

    tim.cr1.write(|w| w.dir().up());

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
        .set_accel_odr(lsm303agr::AccelOutputDataRate::Hz400)
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
    gyro.set_scale(l3gd20::Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz380).unwrap();

    let sens = gyro.scale().unwrap();

    // gyro calibration

    let mut gyro_bias_x = 0.;
    let mut gyro_bias_y = 0.;

    // welford mean
    for i in 1..(1_000) {
        let I16x3 { x: gx, y: gy, .. } = gyro.gyro().unwrap();
        let delta_x = gx as f32 - gyro_bias_x;
        gyro_bias_x += delta_x / i as f32;
        let delta_y = gy as f32 - gyro_bias_y;
        gyro_bias_y += delta_y / i as f32;
    }

    let gyro_bias_x = gyro_bias_x as i16;
    let gyro_bias_y = gyro_bias_y as i16;

    // accel calibration
    let mut accel_calib_ctr = 1;

    let mut accel_bias_x = 0.;
    let mut accel_bias_y = 0.;
    let mut accel_bias_z = 0.;

    loop {
        if accel_calib_ctr > (1_000) {
            break;
        }

        let accel = sensor.accel_data();

        if accel.is_err() {
            continue;
        }

        let Measurement {
            x: ax,
            y: ay,
            z: az,
        } = accel.unwrap();

        let delta_x = ax as f32 - accel_bias_x;
        accel_bias_x += delta_x / accel_calib_ctr as f32;
        let delta_y = ay as f32 - accel_bias_y;
        accel_bias_y += delta_y / accel_calib_ctr as f32;
        let delta_z = az as f32 - accel_bias_z;
        accel_bias_z += delta_z / accel_calib_ctr as f32;

        accel_calib_ctr += 1;
    }

    let accel_bias_x = accel_bias_x as i32;
    let accel_bias_y = accel_bias_y as i32;
    let accel_bias_z = accel_bias_z as i32 - 1000;

    let mut roll = 0.;
    let mut pitch = 0.;

    tim.cr1.write(|w| w.cen().set_bit());

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

        let ax = ax - accel_bias_x;
        let ay = ay - accel_bias_y;
        let az = az - accel_bias_z;

        let I16x3 { x: gx, y: gy, .. } = gyro.gyro().unwrap();
        let t = tim.cnt.read().cnt().bits();
        let dt = t as f32 / CLOCKSPD as f32; // 8 mhz clock ticks to seconds
        tim.cnt.write(|w| w.cnt().bits(0));

        // complementary filter
        // let accel_roll = (ay as f32).atan2(az as f32) * RAD_TO_DEG;
        let accel_roll = atan2f(ay as f32, az as f32) * RAD_TO_DEG;
        let gyro_roll = sens.degrees(gx - gyro_bias_x) * dt + roll;
        roll = ALPHA * gyro_roll + (1. - ALPHA) * accel_roll;

        // let accel_pitch = (ax as f32).atan2(az as f32) * RAD_TO_DEG;
        let accel_pitch = atan2f(ax as f32, az as f32) * RAD_TO_DEG;
        let gyro_pitch = sens.degrees(gy - gyro_bias_y) * dt + pitch;
        pitch = ALPHA * gyro_pitch + (1. - ALPHA) * accel_pitch;

        iprintln!(&mut itm.stim[0], "{} {} {}", roll, pitch, dt);

        delay.delay_us(TS_US);
    }
}
