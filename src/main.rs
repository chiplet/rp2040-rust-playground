//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{entry, hal::pio::PinState};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use hatlet_0_1_0 as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    // let mut led_pin = pins.led.into_push_pull_output();
    let mut led0_pin = pins.led0.into_push_pull_output();
    let mut led1_pin = pins
        .led1
        .into_push_pull_output_in_state(embedded_hal::digital::v2::PinState::High);

    let mut pmod0_pin1 = pins.gpio2.into_push_pull_output();
    let mut pmod0_pin2 = pins.gpio3.into_push_pull_output();

    let mut count = 0u32;

    loop {
        count += 1;
        if count == 50_000 {
            count = 0;
            led0_pin.toggle().unwrap();
            led1_pin.toggle().unwrap();
        }

        pmod0_pin1.set_high().unwrap();
        pmod0_pin2.set_low().unwrap();
        delay.delay_us(1);
        pmod0_pin1.set_low().unwrap();
        pmod0_pin2.set_high().unwrap();
        delay.delay_us(1);
        // info!("on!");
        // led0_pin.set_high().unwrap();
        // led1_pin.set_low().unwrap();
        // delay.delay_ms(500);
        // info!("off!");
        // led0_pin.set_low().unwrap();
        // led1_pin.set_high().unwrap();
        // delay.delay_ms(500);
    }
}

// End of file
