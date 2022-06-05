#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

extern crate alloc;

use core::alloc::Layout;

// use panic_semihosting as _;
use panic_halt as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;
use daisy::led::Led;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use alloc_cortex_m::CortexMHeap;

use core::cell::RefCell;
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
use crate::hal::rcc::rec::I2c1;


use daisy::embedded_hal::digital::v2::OutputPin;
use daisy::embedded_hal::blocking::i2c::*;
use daisy_bsp::hal::adc::AdcSampleTime::{T_1, T_64};
use daisy_bsp::hal::gpio::{Analog, PushPull};
use daisy_bsp::hal::i2c::{PinScl, PinSda};
use daisy_bsp::hal::rcc::CoreClocks;
use daisy_bsp::pins::Pins;

use daisy::pac;
use pac::interrupt;

use daisy::audio;
use adafruit_alphanum4::*;
use ht16k33::{Dimming, Display, HT16K33};
use ht16k33::i2c_mock::I2cMock;
use shared_bus::{BusManager, CortexMMutex, I2cProxy, NullMutex};

use crate::hal::rcc::Ccdr;
use crate::i2c::I2c;
use crate::stm32::{I2C1, Peripherals};
use crate::hal::gpio::Output;
use crate::display_driver::*;
use dsp::osc;

mod display_driver;
mod dsp;

// - static global state ------------------------------------------------------

// static AUDIO_INTERFACE: Mutex<RefCell<Option<audio::Interface>>> = Mutex::new(RefCell::new(None));
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

// static mut SHARED_DEVICE_1:
// Option<MyDevice<shared_bus::I2cProxy<shared_bus::CortexMMutex<SomeI2cBus>>>>
// = None;
static mut SHARED_DEVICE_1:
Option<HT16K33<I2cProxy<'_, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C1>>>>>>
= None;
static mut SHARED_DEVICE_2:
Option<I2c<shared_bus::I2cProxy<shared_bus::CortexMMutex<I2C1>>>>
= None;
static mut SHARED_DEVICE_3:
Option<I2c<shared_bus::I2cProxy<shared_bus::CortexMMutex<I2C1>>>>
= None;

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    //TODO: blink the user LED in a pattern
    // loop {
    //     ;//loggit!("OOM");
    // }
    panic!()
}


// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {

    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = daisy::pac::Peripherals::take().unwrap();
    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let mut pwrcfg = pwr.freeze();
    // // Take the backup power domain
    // let backup = pwrcfg.backup().unwrap();
    // Constrain and Freeze clock
    let mut rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(400.mhz())
        .per_ck(36.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::PER);

    let pins = board.split_gpios(dp.GPIOA.split(ccdr.peripheral.GPIOA),
                                 dp.GPIOB.split(ccdr.peripheral.GPIOB),
                                 dp.GPIOC.split(ccdr.peripheral.GPIOC),
                                 dp.GPIOD.split(ccdr.peripheral.GPIOD),
                                 dp.GPIOE.split(ccdr.peripheral.GPIOE),
                                 dp.GPIOF.split(ccdr.peripheral.GPIOF),
                                 dp.GPIOG.split(ccdr.peripheral.GPIOG));
    let mut delay = Delay::new(cp.SYST, ccdr.clocks);
    let mut led_user = daisy::led::LedUser::new(pins.LED_USER);

    // Initialize the heap allocator
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1_048_576; // in bytes
    unsafe { ALLOCATOR.init(start, size) }


    // let audio_pins = (pins.AK4556.PDN.into_push_pull_output(),
    //             pins.AK4556.MCLK_A.into_alternate_af6(),
    //             pins.AK4556.SCK_A.into_alternate_af6(),
    //             pins.AK4556.FS_A.into_alternate_af6(),
    //             pins.AK4556.SD_A.into_alternate_af6(),
    //             pins.AK4556.SD_B.into_alternate_af6());
    //
    // let sai1_prec = ccdr.peripheral.SAI1.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::PLL3_P);

    //setup i2c1 bus for shared use
    loggit!("Setting up i2c");

    let mut scl = pins.SEED_PIN_11.into_alternate_af4().set_open_drain();
    let mut sda = pins.SEED_PIN_12.into_alternate_af4().set_open_drain();
    let mut i2c1 = dp.I2C1.i2c(
        (scl, sda),
        50.khz(),
        ccdr.peripheral.I2C1,
        &ccdr.clocks,
    );



    // Create a mock I2C device.
    // let mut i2cmock = I2cMock::new();
    // let mut ht16k33 = HT16K33::new(i2cmock, 0x71);

    let i2c1_bus: &'static _ = shared_bus::new_cortexm!(hal::i2c::I2c<daisy_bsp::pac::I2C1> =  i2c1).unwrap();
    let d1b = i2c1_bus.acquire_i2c();
    // let d2b = i2c1_bus.acquire_i2c();
    // let d3b = i2c1_bus.acquire_i2c();
    let mut display_1 = HT16K33::new(d1b, 0x70);
    // let mut display_2 = HT16K33::new(d1b, 0x70);
    // let mut display_3 = HT16K33::new(d1b, 0x70);
    unsafe {
        SHARED_DEVICE_1 = Some(display_1);
        // SHARED_DEVICE_2 = Some(display_2);
        // SHARED_DEVICE_3 = Some(display_3);
    }

    // let combined_display = DisplayDriver::init_display(&i2c1);
    let mut combined_display = DisplayDriver::init_display();
    // let combined_display = DisplayDriver::init_display(d1b,d2b,d3b);
    // let combined_display = DisplayDriver::init_display(i2c1_bus);

    // let audio_interface = audio::Interface::init(&ccdr.clocks,
    //                                              sai1_prec,
    //                                              audio_pins,
    //                                              ccdr.peripheral.DMA1).unwrap();


    // // - audio callback -------------------------------------------------------
    //
    // // handle callback with function pointer
    // #[cfg(not(feature = "alloc"))]
    //     let audio_interface = {
    //     fn callback(fs: f32, block: &mut audio::Block) {
    //         static mut OSC_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    //         static mut OSC_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);
    //         unsafe { OSC_1.dx = (1. / fs) * 110.00 };
    //         unsafe { OSC_2.dx = (1. / fs) * 110.00 };
    //         for frame in block {
    //             *frame = (unsafe { OSC_1.step() },
    //                       unsafe { OSC_2.step() });
    //         }
    //     }
    //
    //     // audio_interface.spawn(callback)
    //     // audio_interface.start(callback)
    // };
    //
    // // handle callback with closure (needs alloc)
    // #[cfg(any(feature = "alloc"))]
    //     let audio_interface = {
    //     let mut osc_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    //     let mut osc_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);
    //
    //     audio_interface.spawn(move |fs, block| {
    //         osc_1.dx = (1. / fs) * 110.00;
    //         osc_2.dx = (1. / fs) * 110.00;
    //         for frame in block {
    //             *frame = (osc_1.step(), osc_2.step());
    //         }
    //     })
    // };

    // let audio_interface = match audio_interface {
    //     Ok(audio_interface) => audio_interface,
    //     Err(e) => {
    //         loggit!("Failed to start audio interface: {:?}", e);
    //         loop {}
    //     }
    // };

    // cortex_m::interrupt::free(|cs| {
    //     AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
    // });


    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().0 / 10;
    let mut ctr = 0u32;

    loop {
        ctr += 1;
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
        combined_display.display_numbers_frame();
        // display_1.update_buffer_with_digit(Index::One, ((ctr + 0) % 10) as u8);
        // display_1.update_buffer_with_digit(Index::Two, ((ctr + 1) % 10) as u8);
        // display_1.update_buffer_with_digit(Index::Three, ((ctr + 2) % 10) as u8);
        // display_1.update_buffer_with_digit(Index::Four, ((ctr + 3) % 10) as u8);
        // display_1.write_display_buffer().unwrap();
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


// - interrupts ---------------------------------------------------------------

// interrupt handler for: dma1, stream1
// #[interrupt]
// fn DMA1_STR1() {
//     cortex_m::interrupt::free(|cs| {
//         if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
//             match audio_interface.handle_interrupt_dma1_str1() {
//                 Ok(()) => (),
//                 Err(e) => {
//                     loggit!("Failed to handle interrupt: {:?}", e);
//                 }
//             };
//         }
//     });
// }
