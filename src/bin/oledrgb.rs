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

// Embed the `Hz` function/trait:
use fugit::RateExtU32;

use bsp::hal::{gpio, spi};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_graphics::{geometry::Point, image::Image, pixelcolor::Rgb565, prelude::*};
use ssd1331::{DisplayRotation::Rotate0, Ssd1331};
use tinybmp::Bmp;

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

    // Set up SPI interface and digital pin. These are stub implementations used in examples.
    let spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio10.reconfigure();
    let spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio11.reconfigure();
    let spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio12.reconfigure();
    let spi_cs = pins.gpio13.into_push_pull_output();

    // Create an SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(), // card initialization happens at low baud rate
        embedded_hal::spi::MODE_0,
    );

    let mut dc = pins.gpio14.into_push_pull_output();
    let mut reset = pins.gpio15.into_push_pull_output();

    let mut display = Ssd1331::new(spi, dc, ssd1331::DisplayRotation::Rotate180);
    display.reset(&mut reset, &mut delay).unwrap();
    display.init().unwrap();
    display.flush().unwrap();

    let (w, h) = display.dimensions();

    let bmp =
        Bmp::from_slice(include_bytes!("./rust-pride.bmp")).expect("Failed to load BMP image");

    let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::zero());

    // Position image in the center of the display
    let moved = im.translate(Point::new(
        (w as u32 - bmp.size().width) as i32 / 2 - 16,
        (h as u32 - bmp.size().height) as i32 / 2,
    ));

    moved.draw(&mut display).unwrap();

    info!("möö :)");
    display.flush().unwrap();

    let xpos = 0;
    let mut t = 0;
    let r = 32;

    loop {
        for y in 0..64 {
            for x in 0..96 {
                let dx = 64 - x;
                let dy = 32 - y;
                if dx * dx + dy * dy > r * r {
                    display.set_pixel(x, y, 0xffff);
                } else {
                    display.set_pixel(x, y, (x + y + t % 96) as u16);
                }
            }
        }
        moved.draw(&mut display).unwrap();
        t += 1;
        display.flush().unwrap();
        // delay.delay_ms(16);
    }
}

// End of file
