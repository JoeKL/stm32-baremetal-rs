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
                $gpio.$pin
                    .into_push_pull_output(&mut $gpio.moder, &mut $gpio.otyper)
                    .downgrade()
                    .downgrade()
            ),*
        ]
    };
}

static BUTTON: Mutex<RefCell<Option<gpioa::PA0<Input>>>> = Mutex::new(RefCell::new(None));

// Operational State Variables
static TRUE_NORTH_BITS: AtomicU32 = AtomicU32::new(0);
static LAST_UNCAL_BITS: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    // Hardware Abstraction Layer (HAL) setup
    let dp = pac::Peripherals::take().expect("Critical: Peripheral access failed");
    let cp = pac::CorePeripherals::take().expect("Critical: Core peripheral access failed");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut exti = dp.EXTI;

    // --- Sensor Initialization (I2C1) ---
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
    sensor.init().expect("Sensor Init Failed: Check I2C wiring");

    // --- Magnetometer Configuration ---
    // High Resolution mode ensures 16-bit precision for safety-critical navigation
    sensor
        .set_mag_mode_and_odr(
            &mut delay,
            lsm303agr::MagMode::HighResolution,
            lsm303agr::MagOutputDataRate::Hz10,
        )
        .unwrap();

    let Ok(mut sensor) = sensor.into_mag_continuous() else {
        panic!("Driver Error: Could not enter continuous sampling mode")
    };
    sensor.mag_enable_low_pass_filter().unwrap();

    // User Interface: PA0 Blue Button for field calibration
    let mut btn_0 = gpioa
        .pa0
        .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

    btn_0.trigger_on_edge(&mut exti, hal::gpio::Edge::Rising);
    btn_0.enable_interrupt(&mut exti);

    cortex_m::interrupt::free(|cs| {
        BUTTON.borrow(cs).replace(Some(btn_0));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI0);
    }

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

    loop {
        // Poll for new magnetometer data
        let mag_ready = sensor
            .mag_status()
            .map(|s| s.xyz_new_data())
            .unwrap_or(false);

        if mag_ready {
            let mag_data = sensor.magnetic_field().unwrap().xyz_unscaled();

            // Perform heading calculation via Safety-Wrapped C library
            // This handles Hard-Iron offsets and avoids undefined behavior (NaN/Inf)
            match wrapper::wrapper::safe_calc_heading_in_rad(mag_data.0, mag_data.1) {
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

                    hprintln!("Heading: {}°", degrees);

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
        }
    }
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        let mut btn_ref = BUTTON.borrow(cs).borrow_mut();
        if let Some(ref mut btn) = *btn_ref {
            // CRITICAL: You MUST clear the interrupt pending bit,
            // otherwise the CPU will jump back here immediately!
            btn.clear_interrupt();

            let last_degrees_uncal = f32::from_bits(LAST_UNCAL_BITS.load(Ordering::Relaxed));

            hprintln!(
                "Calibration Triggered: New North Offset: {}°",
                last_degrees_uncal
            );

            TRUE_NORTH_BITS.store(last_degrees_uncal.to_bits(), Ordering::Relaxed);
        }
    });
}
