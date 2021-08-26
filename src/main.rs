#![no_std]
#![no_main]

use core::convert::TryInto;
use core::f32::consts::PI;
use cortex_m::iprintln;
use cortex_m_rt::entry;
use hal::{
    delay::Delay,
    spi::{config::Config, Spi},
};
use l3gd20::{I16x3, L3gd20, Odr};
use lsm303agr::{Lsm303agr, Measurement};
// use micromath::F32Ext;
use panic_itm as _;
// use stm32disc::{Direction, Led, Leds};
use stm32f3xx_hal::{self as hal, pac, prelude::*};

const TS_MS: u32 = 100;
const TS_S: f32 = 0.1;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;
    let mut syst = cp.SYST;

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
    sensor.set_accel_scale(lsm303agr::AccelScale::G4).unwrap();
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
    gyro.set_scale(l3gd20::Scale::Dps500).unwrap();
    gyro.set_odr(Odr::Hz190).unwrap();

    let sens = gyro.scale().unwrap();

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

        let I16x3 {
            x: gx,
            y: gy,
            z: gz,
        } = gyro.gyro().unwrap();

        iprintln!(
            &mut itm.stim[0],
            "{} {} {} {} {} {}",
            ax + 40,
            ay,
            az - 1000,
            sens.degrees(gx) * TS_S,
            sens.degrees(gy) * TS_S,
            sens.degrees(gz) * TS_S
        );

        delay.delay_ms(TS_MS);
    }
}
