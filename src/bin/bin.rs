#![no_std]
#![no_main]

use panic_halt as _;

use stm32f3xx_hal as hal;

use hal::adc::Adc;
use hal::adc::CommonAdc;
use hal::adc::config::Config;
use hal::{pac, prelude::*, pwm::tim2};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Critical: Peripheral access failed");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let config = Config::default();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    // ADC Setup für F303
    let common_adc = CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);
    let mut adc1 = Adc::new(dp.ADC1, config, &clocks, &common_adc);

    let mut vr_x_pin = gpioc.pc2.into_analog(&mut gpioc.moder, &mut gpioc.pupdr); // PC2 -> VRx
    // let mut vr_y_pin = gpioc.pc3.into_analog(&mut gpioc.moder, &mut gpioc.pupdr); // PC3 -> VRy

    let (_, ch2_no_pins, _, _) = tim2(dp.TIM2, 20_000, 50.Hz(), &clocks);

    //Servo: PA1 -> ServoPWM
    let pa1 = gpioa
        .pa1
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mut servo_channel = ch2_no_pins.output_to_pa1(pa1);
    servo_channel.enable();

    loop {
        let raw_x_value: u16 = adc1.read(&mut vr_x_pin).expect("Messung fehlgeschlagen");
        // let raw_y_value: u16 = adc1.read(&mut vr_y_pin).expect("Messung fehlgeschlagen");

        let mut norm_x_value: i16 = (raw_x_value as f32 - 2048.0) as i16;
        // let norm_y_value: i16 = (raw_y_value as f32 - 2048.0) as i16;

        let threshold = 0;
        let threshold_range = -threshold..threshold;

        if !threshold_range.contains(&norm_x_value) {
            // 0. invert x
            norm_x_value *= -1;

            // 1. Shift joystick from [-2048..2047] to [0..4095]
            let x_positive = norm_x_value as i32 + 2048;

            // 2. Scale the range (Multiply by target range of 2000, divide by input range of 4095)
            let scaled = (x_positive * 2000) / 4095;

            // 3. Add the servo offset (500)
            let duty_cycle = scaled + 500;

            // 4. Clamp it just to be totally safe, then cast to u32 for the HAL
            let safe_duty = duty_cycle.clamp(500, 2500) as u32;

            servo_channel.set_duty(safe_duty);
        } else {
            // If the stick is in the deadzone, snap the servo exactly to the center (1500)
            servo_channel.set_duty(1500);
        }
    }
}
