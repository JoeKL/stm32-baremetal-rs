//! Target board: STM32F3DISCOVERY
//! Magnetic Compass Application for STM32F3 Discovery
//!
//! This application implements a digital compass using the onboard LSM303AGR
//! sensor. It demonstrates a hybrid Rust/C architecture, leveraging legacy C
//! math logic within a memory-safe Rust wrapper, aligned with ISO 25119
//! safety principles for agricultural equipment.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use cortex_m::interrupt::Mutex;
use lsm303agr;
use panic_halt as _;
use stm32_baremetal::ffi::wrapper;

use hal::interrupt;
use stm32f3xx_hal as hal;
use stm32f3xx_hal::adc::Adc;
use stm32f3xx_hal::adc::CommonAdc;
use stm32f3xx_hal::adc::config::Config;
use stm32f3xx_hal::gpio::Input;
use stm32f3xx_hal::gpio::gpioa;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use hal::pac;
use hal::prelude::*;

/// Initializes a circular array of LEDs by "downgrading" specific GPIO types
/// into a common type. This allows for runtime indexing of the compass display.
macro_rules! init_leds {
    ($gpio:expr, [$($pin:ident),*]) => {
        [
            $(
                init_led!($gpio,$pin)
            ),*
        ]
    };
}

macro_rules! init_led {
    ($gpio:expr, $pin:ident) => {
        $gpio
            .$pin
            .into_push_pull_output(&mut $gpio.moder, &mut $gpio.otyper)
            .downgrade()
            .downgrade()
    };
}

// Operational State Variables
static TRUE_NORTH_BITS: AtomicU32 = AtomicU32::new(0);
static LAST_UNCAL_BITS: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Critical: Peripheral access failed");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let config = Config::default();

    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    // init leds
    // LED Compass Mapping (Clockwise)
    // Index | Pin  | Label | Color  | Direction
    // -----------------------------------------
    // [0]   | PE9  | LD3   | Red    | North
    // [1]   | PE10 | LD5   | Orange | North-East
    // [2]   | PE11 | LD7   | Green  | East
    // [3]   | PE12 | LD9   | Blue   | South-East
    // [4]   | PE13 | LD10  | Red    | South
    // [5]   | PE14 | LD8   | Orange | South-West
    // [6]   | PE15 | LD6   | Green  | West
    // [7]   | PE8  | LD4   | Blue   | North-West
    let mut led_array = init_leds!(gpioe, [pe9, pe10, pe11, pe12, pe13, pe14, pe15, pe8]);

    // ADC Setup für F303
    let common_adc = CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);
    let mut adc1 = Adc::new(dp.ADC1, config, &clocks, &common_adc);

    let mut vr_x_pin = gpioc.pc2.into_analog(&mut gpioc.moder, &mut gpioc.pupdr); // PC2 -> VRx
    let mut vr_y_pin = gpioc.pc3.into_analog(&mut gpioc.moder, &mut gpioc.pupdr); // PC3 -> VRy

    loop {
        let raw_x_value: u16 = adc1.read(&mut vr_x_pin).expect("Messung fehlgeschlagen");
        let raw_y_value: u16 = adc1.read(&mut vr_y_pin).expect("Messung fehlgeschlagen");

        let norm_x_value: i16 = (raw_x_value as f32 - 2048.0) as i16;
        let norm_y_value: i16 = (raw_y_value as f32 - 2048.0) as i16;

        let threshold = 100;
        let threshold_range = -threshold..threshold;

        if !threshold_range.contains(&norm_x_value) || !threshold_range.contains(&norm_y_value) {
            // Perform heading calculation via Safety-Wrapped C library
            // This handles Hard-Iron offsets and avoids undefined behavior (NaN/Inf)
            match wrapper::wrapper::safe_calc_heading_in_rad(norm_x_value, norm_y_value) {
                Ok(heading_rad) => {
                    // rad to degrees and add offset to calibrate
                    let degrees_uncal = heading_rad * (180.0 / core::f32::consts::PI);
                    LAST_UNCAL_BITS.store(degrees_uncal.to_bits(), Ordering::Relaxed);

                    // Apply user-defined North calibration offset
                    let mut degrees =
                        degrees_uncal - f32::from_bits(TRUE_NORTH_BITS.load(Ordering::Relaxed));

                    // Normalize angle to 0.0 <= x < 360.0 range
                    if degrees < 0.0 {
                        degrees += 360.0;
                    }
                    if degrees >= 360.0 {
                        degrees -= 360.0;
                    }

                    // Convert degrees to LED index (8 LEDs = 45° segments)
                    // Added 22.5° offset to center the LED on the cardinal direction
                    let led_index = (((degrees + 22.5) % 360.0) / 45.0) as usize;

                    // Update LED display (Clear all, set active)
                    for led in led_array.iter_mut() {
                        led.set_low().ok();
                    }
                    led_array[led_index % 8].set_high().ok();
                }
                Err(e) => {
                    // Fail-safe: Log error. In production, this would trigger a system alarm.
                    hprintln!("Safety Fault: {:?}", e);
                }
            }
        } else {
            // Update LED display (Clear all, set active)
            for led in led_array.iter_mut() {
                led.set_low().ok();
            }
        }
    }
}
