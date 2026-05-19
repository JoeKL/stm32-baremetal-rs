#![no_std]
#![no_main]

use panic_halt as _;

use stm32f3xx_hal as hal;

use hal::adc::Adc;
use hal::adc::CommonAdc;
use hal::adc::config::Config;
use hal::{pac, prelude::*, pwm::tim2};

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Critical: Peripheral access failed");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let config = Config::default();

    let clocks = rcc.cfgr.sysclk(64.MHz()).freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    // ADC Setup für F303
    let common_adc = CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);
    let mut adc1 = Adc::new(dp.ADC1, config, &clocks, &common_adc);

    let mut vr_x_pin = gpioc.pc2.into_analog(&mut gpioc.moder, &mut gpioc.pupdr); // PA2 -> VRx
    let mut vr_y_pin = gpioc.pc3.into_analog(&mut gpioc.moder, &mut gpioc.pupdr); // PA3 -> VRy

    let (_, ch2_no_pins, _, _) = tim2(dp.TIM2, 20_000, 50.Hz(), &clocks);

    let pa1 = gpioa
        .pa1
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut servo_channel = ch2_no_pins.output_to_pa1(pa1);

    servo_channel.enable();

    hprintln!("resetting servo");
    servo_channel.set_duty(1500);
    cortex_m::asm::delay(16_000_000);

    loop {
        let raw_x_value: u16 = adc1.read(&mut vr_x_pin).expect("Messung fehlgeschlagen");
        cortex_m::asm::delay(10_000);
        let raw_y_value: u16 = adc1.read(&mut vr_y_pin).expect("Messung fehlgeschlagen");

        let norm_x_value: i16 = (raw_x_value as f32 - 2048.0) as i16;
        let norm_y_value: i16 = (raw_y_value as f32 - 2048.0) as i16;

        let threshold = 0;
        let threshold_range = -threshold..threshold;

        if !threshold_range.contains(&norm_x_value) || !threshold_range.contains(&norm_y_value) {
            hprintln!("{} {}", norm_x_value, norm_y_value);
        }

        // // Move to 0 degrees
        // servo_channel.set_duty(500);
        // cortex_m::asm::delay(16_000_000);
        //
        // // Move to 90 degrees
        // servo_channel.set_duty(1500);
        // cortex_m::asm::delay(16_000_000);
        //
        // // Move to 180 degrees
        // servo_channel.set_duty(2500);
        // cortex_m::asm::delay(16_000_000);
        //
        // servo_channel.set_duty(1500);
        // cortex_m::asm::delay(16_000_000);

        cortex_m::asm::delay(640_000);
    }
}
