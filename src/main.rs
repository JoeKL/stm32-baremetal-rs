//! Target board: STM32F3DISCOVERY

#![no_main]
#![no_std]

use lsm303agr;
use micromath::F32Ext;
use panic_halt as _;

use stm32f3xx_hal as hal;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use hal::pac;
use hal::prelude::*;

// use macro to init compass leds
macro_rules! init_leds {
    ($gpio:expr, [$($pin:ident),*]) => {
        [
            $(
                $gpio.$pin
                    .into_push_pull_output(&mut $gpio.moder, &mut $gpio.otyper)
                    .downgrade()
                    .downgrade()
            ),*
        ]
    };
}

fn calc_heading_in_rad(mx_data: i16, my_data: i16) -> f32 {
    let mx = mx_data as f32;
    let my = my_data as f32;

    let mx_c = mx - 113.0;
    let my_c = my - (-263.0);

    let heading_rad = my_c.atan2(mx_c);

    heading_rad
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Failed to take peripherals");
    let cp = pac::CorePeripherals::take().expect("Failed to take core peripherals");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut exti = dp.EXTI;

    // init sensor
    let mut scl =
        gpiob
            .pb6
            .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    let mut sda =
        gpiob
            .pb7
            .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    scl.internal_pull_up(&mut gpiob.pupdr, true);
    sda.internal_pull_up(&mut gpiob.pupdr, true);

    let i2c = hal::i2c::I2c::new(
        dp.I2C1,
        (scl, sda),
        100.kHz().try_into().unwrap(),
        clocks,
        &mut rcc.apb1,
    );

    let mut sensor = lsm303agr::Lsm303agr::new_with_i2c(i2c);
    sensor.init().expect("Failed to initialize the AGR sensor");

    // configure sensor
    sensor
        .set_mag_mode_and_odr(
            &mut delay,
            lsm303agr::MagMode::HighResolution,
            lsm303agr::MagOutputDataRate::Hz10,
        )
        .unwrap();

    let Ok(mut sensor) = sensor.into_mag_continuous() else {
        panic!("Error enabling continuous mode")
    };
    sensor.mag_enable_low_pass_filter().unwrap();

    // init button
    let mut btn_0 = gpioa
        .pa0
        .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

    btn_0.trigger_on_edge(&mut exti, hal::gpio::Edge::Rising);

    // init leds
    let mut led_array = init_leds!(gpioe, [pe9, pe10, pe11, pe12, pe13, pe14, pe15, pe8]);

    let mut last_degrees_uncal = 0.0;
    let mut true_north_degrees = 0.0;

    loop {
        let mag_ready = sensor
            .mag_status()
            .map(|s| s.xyz_new_data())
            .unwrap_or(false);

        if mag_ready {
            let mag_data = sensor.magnetic_field().unwrap().xyz_unscaled();

            let heading_rad = calc_heading_in_rad(mag_data.0, mag_data.1);

            // rad to degrees and add offset to calibrate
            let degrees_uncal = heading_rad * (180.0 / core::f32::consts::PI);
            last_degrees_uncal = degrees_uncal;
            let mut degrees = degrees_uncal - true_north_degrees;

            // clamp the overflow
            if degrees < 0.0 {
                degrees += 360.0;
            }
            if degrees >= 360.0 {
                degrees -= 360.0;
            }

            hprintln!("degrees = {}°", degrees);

            // map to LED (45 degrees per LED)
            let led_index = (((degrees + 22.5) % 360.0) / 45.0) as usize;

            for led in led_array.iter_mut() {
                led.set_low().ok();
            }
            led_array[led_index % 8].set_high().ok();
        }

        // button to calibrate
        if btn_0.is_high().unwrap() {
            hprintln!("Calibration: offset is now = {}°", last_degrees_uncal);
            true_north_degrees = last_degrees_uncal;
        }
    }
}
