#![no_std]
#![no_main]

use panic_halt as _;

use stm32f3xx_hal as hal;

use hal::{pac, prelude::*, pwm::tim2};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Critical: Peripheral access failed");

    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    let clocks = rcc.cfgr.sysclk(64.MHz()).freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let (_, ch2_no_pins, _, _) = tim2(dp.TIM2, 20_000, 50.Hz(), &clocks);

    let pa1 = gpioa
        .pa1
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut servo_channel = ch2_no_pins.output_to_pa1(pa1);

    servo_channel.enable();

    servo_channel.set_duty(1500);
    cortex_m::asm::delay(16_000_000);

    loop {
        // Move to 0 degrees
        servo_channel.set_duty(500);
        cortex_m::asm::delay(16_000_000);

        // Move to 90 degrees
        servo_channel.set_duty(1500);
        cortex_m::asm::delay(16_000_000);

        // Move to 180 degrees
        servo_channel.set_duty(2500);
        cortex_m::asm::delay(16_000_000);

        servo_channel.set_duty(1500);
        cortex_m::asm::delay(16_000_000);
    }
}
