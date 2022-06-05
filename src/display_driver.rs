use core::borrow::Borrow;
use core::cell::RefCell;
use core::fmt::Debug;
use daisy_bsp as daisy;
use daisy::hal;
use daisy_bsp::loggit;
use hal::prelude::*;
use hal::pac::RTC;
use hal::pac::rtc;
use hal::rcc::rec::AdcClkSel;
use hal::rcc::rec::I2c123ClkSel;
use hal::rcc::ResetEnable;
use hal::adc;
use hal::delay::Delay;
use hal::i2c;
use hal::stm32;
use daisy::embedded_hal::blocking::i2c::*;
use crate::hal::rcc::rec::I2c1;

use adafruit_alphanum4::*;
use ht16k33::{Dimming, Display, HT16K33};
use shared_bus::{BusManager, BusMutex, CortexMMutex, I2cProxy, NullMutex};

use crate::hal::rcc::Ccdr;
use crate::i2c::I2c;
use crate::stm32::{I2C1, Peripherals};
use crate::hal::gpio::Output;
use crate::{Mutex, SHARED_DEVICE_1};


pub struct DisplayDriver<'a> {
    // i2c: I2C,
    // bus: BusManager<NullMutex<I2C>>,
    display_1: &'a mut HT16K33<I2cProxy<'a, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C1>>>>>,
    // display_2: HT16K33<I2cProxy<'a, NullMutex<I2C>>>,
    // display_3: HT16K33<I2cProxy<'a, NullMutex<I2C>>>,
    counter: u32,
}

impl DisplayDriver<'static>
where
{
    // pub fn init_display(i2c: &I2C) -> Self // Result<Self, E>
    // pub fn init_display(i2c1_bus: BusManager<NullMutex<I2C>>) -> Self
    // pub fn init_display(d1b:I2cProxy<NullMutex<I2C>>,d2b:I2cProxy<NullMutex<I2C>>,d3b:I2cProxy<NullMutex<I2C>>) -> Self
    // pub fn init_display(mut display_1: &'static mut HT16K33<I2cProxy<CortexMMutex<I2C>>>) -> Self
    pub fn init_display() -> Self
                        // mut display_2: HT16K33<I2cProxy<NullMutex<I2C>>>,
                        // mut display_3: HT16K33<I2cProxy<NullMutex<I2C>>>
    {
        // let i2c1_bus = shared_bus::BusManagerSimple::new(*i2c);

        let display_1 = unsafe {SHARED_DEVICE_1.as_mut().unwrap()};

        loggit!("Inited driver");
        // let d1b = i2c1_bus.acquire_i2c();
        // let mut display_1 = HT16K33::new(d1b, 0x70);
        loggit!("Created display");
        display_1.initialize().expect("Failed to initialize ht16k33");
        loggit!("Initialized display");
        display_1.set_display(Display::ON).expect("Could not turn on the display!");
        display_1.set_dimming(Dimming::BRIGHTNESS_3_16).expect("Could not set brightness");
        display_1.write_display_buffer().unwrap();
        loggit!("Updated buffer");

        // // let d2b = i2c1_bus.acquire_i2c();
        // // let mut display_2 = HT16K33::new(d2b, 0x71);
        // display_2.initialize().expect("Failed to initialize ht16k33");
        // display_2.set_display(Display::ON).expect("Could not turn on the display!");
        // display_2.set_dimming(Dimming::BRIGHTNESS_3_16).expect("Could not set brightness");
        // display_2.write_display_buffer().unwrap();
        //
        // // let d3b = i2c1_bus.acquire_i2c().clone();
        // // let mut display_3 = HT16K33::new(d3b, 0x72);
        // display_3.initialize().expect("Failed to initialize ht16k33");
        // display_3.set_display(Display::ON).expect("Could not turn on the display!");
        // display_3.set_dimming(Dimming::BRIGHTNESS_3_16).expect("Could not set brightness");
        // display_3.write_display_buffer().unwrap();

        let mut dd = DisplayDriver {
            // i2c: I2C,
            // bus: i2c1_bus,
            display_1,
            // display_2,
            // display_3,
            counter: 0,
        };
        dd
    }

    pub fn display_numbers_frame(&mut self) {
        self.counter += 1;
        self.display_1.update_buffer_with_digit(Index::One, ((self.counter + 0) % 10) as u8);
        self.display_1.update_buffer_with_digit(Index::Two, ((self.counter + 1) % 10) as u8);
        self.display_1.update_buffer_with_digit(Index::Three, ((self.counter + 2) % 10) as u8);
        self.display_1.update_buffer_with_digit(Index::Four, ((self.counter + 3) % 10) as u8);
        self.display_1.write_display_buffer().unwrap();
        // display_2.update_buffer_with_digit(Index::One, ((ctr + 4) % 10) as u8);
        // display_2.update_buffer_with_digit(Index::Two, ((ctr + 5) % 10) as u8);
        // display_2.update_buffer_with_digit(Index::Three, ((ctr + 6) % 10) as u8);
        // display_2.update_buffer_with_digit(Index::Four, ((ctr + 7) % 10) as u8);
        // display_2.write_display_buffer().unwrap();
        // display_3.update_buffer_with_digit(Index::One, ((ctr + 8) % 10) as u8);
        // display_3.update_buffer_with_digit(Index::Two, ((ctr + 9) % 10) as u8);
        // display_3.update_buffer_with_digit(Index::Three, ((ctr + 10) % 10) as u8);
        // display_3.update_buffer_with_digit(Index::Four, ((ctr + 11) % 10) as u8);
        // display_3.write_display_buffer().unwrap();
        // loggit!("{}", ctr);

    }
}
