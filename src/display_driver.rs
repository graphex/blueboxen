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
use crate::{Mutex, SHARED_DEVICE_1, SHARED_DEVICE_2, SHARED_DEVICE_3};


pub struct DisplayDriver<'a> {
    display_1: &'a mut HT16K33<I2cProxy<'a, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C1>>>>>,
    display_2: &'a mut HT16K33<I2cProxy<'a, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C1>>>>>,
    display_3: &'a mut HT16K33<I2cProxy<'a, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C1>>>>>,
    counter: u32,
}

impl DisplayDriver<'static>
where
{
    pub fn init_display() -> Self
    {
        let display_1 = unsafe {SHARED_DEVICE_1.as_mut().unwrap()};
        let display_2 = unsafe {SHARED_DEVICE_2.as_mut().unwrap()};
        let display_3 = unsafe {SHARED_DEVICE_3.as_mut().unwrap()};
        let brightness = Dimming::BRIGHTNESS_1_16;

        display_1.initialize().expect("Failed to initialize ht16k33");
        display_1.set_display(Display::ON).expect("Could not turn on the display!");
        display_1.set_dimming(brightness).expect("Could not set brightness");
        display_1.write_display_buffer().unwrap();
        display_2.initialize().expect("Failed to initialize ht16k33");
        display_2.set_display(Display::ON).expect("Could not turn on the display!");
        display_2.set_dimming(brightness).expect("Could not set brightness");
        display_2.write_display_buffer().unwrap();
        display_3.initialize().expect("Failed to initialize ht16k33");
        display_3.set_display(Display::ON).expect("Could not turn on the display!");
        display_3.set_dimming(brightness).expect("Could not set brightness");
        display_3.write_display_buffer().unwrap();
        loggit!("Set Up Displays");

        let mut dd = DisplayDriver {
            display_1,
            display_2,
            display_3,
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
        self.display_2.update_buffer_with_digit(Index::One, ((self.counter + 4) % 10) as u8);
        self.display_2.update_buffer_with_digit(Index::Two, ((self.counter + 5) % 10) as u8);
        self.display_2.update_buffer_with_digit(Index::Three, ((self.counter + 6) % 10) as u8);
        self.display_2.update_buffer_with_digit(Index::Four, ((self.counter + 7) % 10) as u8);
        self.display_2.write_display_buffer().unwrap();
        self.display_3.update_buffer_with_digit(Index::One, ((self.counter + 8) % 10) as u8);
        self.display_3.update_buffer_with_digit(Index::Two, ((self.counter + 9) % 10) as u8);
        self.display_3.update_buffer_with_digit(Index::Three, ((self.counter + 10) % 10) as u8);
        self.display_3.update_buffer_with_digit(Index::Four, ((self.counter + 11) % 10) as u8);
        self.display_3.write_display_buffer().unwrap();
        loggit!("{}", self.counter);
    }
}
