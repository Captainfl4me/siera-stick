#![no_std]
#![no_main]

mod waveshare_rp2040_zero;

use alloc::string::String;
use core::fmt::Write;
use core::{iter::once, panic::PanicInfo};
use embedded_hal::timer::CountDown;
use fugit::{ExtU32, Rate, RateExtU32};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use waveshare_rp2040_zero::entry;
use waveshare_rp2040_zero::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionI2C, Pin, PullUp, AnyPin},
        i2c::I2C,
        multicore::{Multicore, Stack},
        pac,
        pio::PIOExt,
        timer::Timer,
        usb::UsbBus,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

extern crate alloc;
use alloc::{boxed::Box, rc::Rc};

use slint::platform::{software_renderer::MinimalSoftwareWindow, Platform};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

slint::include_modules!();

struct CustomPlatform {
    window: Rc<MinimalSoftwareWindow>,
    timer: Timer,
}

impl Platform for CustomPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_counter().ticks())
    }
}

const COLOR_THRESHOLD: u8 = 10;
#[derive(Clone, Copy)]
struct BinaryPixel {
    value: bool,
}
impl slint::platform::software_renderer::TargetPixel for BinaryPixel {
    fn blend(&mut self, color: slint::platform::software_renderer::PremultipliedRgbaColor) {
        if color.alpha < 128 {
            return;
        }

        if color.red < u8::MAX - COLOR_THRESHOLD
            || color.green < u8::MAX - COLOR_THRESHOLD
            || color.blue < u8::MAX - COLOR_THRESHOLD
        {
            self.value = true;
        } else {
            self.value = false;
        }
    }

    fn from_rgb(red: u8, green: u8, blue: u8) -> Self {
        if red > COLOR_THRESHOLD || green > COLOR_THRESHOLD || blue > COLOR_THRESHOLD {
            BinaryPixel { value: true }
        } else {
            BinaryPixel { value: false }
        }
    }
}

#[entry]
fn main() -> ! {
    // Create Heap
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 16 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gp0.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gp1.reconfigure();
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let mut display = Ssd1306::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate180,
    )
    .into_buffered_graphics_mode();
    display.init().unwrap();

    // Clear the display before starting
    display.clear_buffer();
    display.flush().unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(&timer, &clocks.peripheral_clock.freq(), pins.neopixel.into_function())
    });

    let window = MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::SwappedBuffers,
    );
    slint::platform::set_platform(Box::new(CustomPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    const DISPLAY_WIDTH: usize = 128;
    const DISPLAY_HEIGHT: usize = 64;
    let mut buffer = [BinaryPixel { value: false }; DISPLAY_WIDTH * DISPLAY_HEIGHT];

    let _ui = MainUI::new();
    window.set_size(slint::PhysicalSize::new(
        DISPLAY_WIDTH as u32,
        DISPLAY_HEIGHT as u32,
    ));

    loop {
        slint::platform::update_timers_and_animations();

        window.draw_if_needed(|renderer| {
            renderer.render(&mut buffer, DISPLAY_WIDTH);
            for x in 0..DISPLAY_WIDTH {
                for y in 0..DISPLAY_HEIGHT {
                    let pixel = buffer[y * DISPLAY_WIDTH + x];
                    display.set_pixel(x as u32, y as u32, pixel.value);
                }
            }
            display.flush().unwrap();
        });
    }
}

static mut CORE1_STACK: Stack<1024> = Stack::new();
fn core1_task(
    timer: &Timer,
    peripheral_clock_freq: &Rate<u32, 1, 1>,
    neopixel: impl AnyPin<Function = <pac::PIO0 as PIOExt>::PinFunction>,
) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };

    let mut delay = timer.count_down();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        neopixel,
        &mut pio,
        sm0,
        *peripheral_clock_freq,
        timer.count_down(),
    );

    // Infinite colour wheel loop
    let mut n = 128;
    loop {
        ws.write(brightness(once(wheel(n)), 100)).unwrap();
        n = n.wrapping_add(1);

        delay.start(25.millis());
        let _ = nb::block!(delay.wait());
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut sio = Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let _ = loop {
            cortex_m::asm::nop();
        };
    });

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // Use serial over USB
    let mut serial = SerialPort::new(&usb_bus);
    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("SiERA")
        .product("RP")
        .serial_number("RP01")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    while timer.get_counter().ticks() < 1_000_000 {
        usb_dev.poll(&mut [&mut serial]);
    }
    let _ = serial.write(b"-- PANIC mode --\r\n");
    (0..10).for_each(|_| {
        usb_dev.poll(&mut [&mut serial]);
    });
    //
    let mut err_msg = String::new();
    let _ = write!(&mut err_msg, "Panic: {:?}\r\n", info);
    err_msg.as_bytes().chunks(127).for_each(|bytes| {
        let _ = serial.write(bytes);
        (0..10).for_each(|_| {
            usb_dev.poll(&mut [&mut serial]);
        });
    });

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut delay = timer.count_down();
    loop {
        // Turn off the LED (black)
        let _ = ws.write(brightness(once(RGB8::new(0, 0, 0)), 0));
        delay.start(200.millis());
        let _ = nb::block!(delay.wait());
        // Turn on the LED (white)
        let _ = ws.write(brightness(once(RGB8::new(255, 0, 0)), 100));
        delay.start(200.millis());
        let _ = nb::block!(delay.wait());
    }
}
