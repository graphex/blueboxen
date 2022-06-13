#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::boxed::Box;
use core::alloc::Layout;
use core::borrow::BorrowMut;

// use panic_semihosting as _;
use panic_halt as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;
use daisy::led::Led;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use alloc_cortex_m::CortexMHeap;

use core::cell::{Cell, RefCell};
use daisy::hal;
use daisy_bsp::{embedded_hal, loggit};
use hal::prelude::*;
use hal::pac::RTC;
use hal::pac::rtc;
use hal::rcc::rec::AdcClkSel;
use hal::rcc::ResetEnable;
use hal::adc;
use hal::delay::Delay;
use hal::i2c;
use hal::stm32;
use crate::hal::rcc::rec::I2c4;


use daisy::embedded_hal::blocking::i2c::*;
use daisy_bsp::hal::adc::AdcSampleTime::{T_1, T_64};
use daisy_bsp::hal::gpio::{Analog, PushPull};
use daisy_bsp::hal::i2c::{PinScl, PinSda};
use daisy_bsp::hal::rcc::{CoreClocks, PllConfigStrategy};
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
use crate::stm32::{I2C4, Peripherals};
use crate::hal::gpio::{Input, Output, PullUp, OpenDrain};
use daisy_bsp::embedded_hal::digital::v2::{InputPin, OutputPin};
use crate::display_driver::*;

mod display_driver;
mod dsp;

use dsp::osc;

// #[macro_use]
// extern crate keypad;
// use crate::hal::gpio::gpioc::PC;
// use crate::hal::gpio::gpiod::PD;
// // - set up keypad (TODO: Move to module) -----------------------------
//
// keypad_struct!{
//     pub struct BlueboxenKeys {
//         rows: (
//             hal::gpio::gpiob::PB4<Input<PullUp>>,
//             // PC<Input<PullUp>>,
//             // PD<Input<PullUp>>,
//             // PD<Input<PullUp>>,
//             // mock_hal::gpioa::PA4<Input<PullUp>>,
//             // mock_hal::gpioa::PA5<Input<PullUp>>,
//             // mock_hal::gpioa::PA6<Input<PullUp>>,
//         ),
//         columns: (
//             hal::gpio::gpioc::PC11<Output<OpenDrain>>,
//             // hal::gpio::gpiog::PG11<Output<OpenDrain>>,
//             // hal::gpio::gpiob::PB4<Output<OpenDrain>>,
//             // hal::gpio::gpiob::PB5<Output<OpenDrain>>,
//             // mock_hal::gpioa::PA7<MockOutput<OpenDrain>>,
//             // mock_hal::gpioa::PA8<MockOutput<OpenDrain>>,
//             // mock_hal::gpioa::PA9<MockOutput<OpenDrain>>,
//             // mock_hal::gpioa::PA10<MockOutput<OpenDrain>>,
//         ),
//     }
// }


// - static global state ------------------------------------------------------

static AUDIO_INTERFACE: Mutex<RefCell<Option<audio::Interface>>> = Mutex::new(RefCell::new(None));

// #[global_allocator]
// static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

static mut SHARED_DEVICE_1:
Option<HT16K33<I2cProxy<'_, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C4>>>>>>
= None;
static mut SHARED_DEVICE_2:
Option<HT16K33<I2cProxy<'_, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C4>>>>>>
= None;
static mut SHARED_DEVICE_3:
Option<HT16K33<I2cProxy<'_, Mutex<RefCell<daisy_bsp::hal::i2c::I2c<I2C4>>>>>>
= None;


static TONE_A: Mutex<Cell<Option<f32>>> = Mutex::new(Cell::new(None));
static TONE_B: Mutex<Cell<Option<f32>>> = Mutex::new(Cell::new(None));


// #[alloc_error_handler]
// fn oom(_: Layout) -> ! {
//     //TODO: blink the user LED in a pattern
//     // loop {
//     //     ;//loggit!("OOM");
//     // }
//     panic!()
// }


// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {

    // - board setup ----------------------------------------------------------
    let cp = cortex_m::Peripherals::take().unwrap();

    // let board = daisy::Board::take().unwrap();
    // let dp = daisy::pac::Peripherals::take().unwrap();
    // // Constrain and Freeze power
    // let pwr = dp.PWR.constrain();
    // let mut pwrcfg = pwr.freeze();
    // // // Take the backup power domain
    // // let backup = pwrcfg.backup().unwrap();
    // // Constrain and Freeze clock
    // let mut ccdr = dp.RCC.constrain()
    //     .use_hse(16.mhz())                               // external crystal @ 16 MHz
    //     .pll1_strategy(PllConfigStrategy::Iterative)  // pll1 drives system clock
    //     .sys_ck(480.mhz())
    //     .per_ck(4.mhz())
    //     .freeze(pwrcfg, &dp.SYSCFG);

    let board = daisy::Board::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    let mut ccdr = board.freeze_clocks(dp.PWR.constrain(),
                                       dp.RCC.constrain(),
                                       &dp.SYSCFG);

    // switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::PER);

    let pins = board.split_gpios(dp.GPIOA.split(ccdr.peripheral.GPIOA),
                                 dp.GPIOB.split(ccdr.peripheral.GPIOB),
                                 dp.GPIOC.split(ccdr.peripheral.GPIOC),
                                 dp.GPIOD.split(ccdr.peripheral.GPIOD),
                                 dp.GPIOE.split(ccdr.peripheral.GPIOE),
                                 dp.GPIOF.split(ccdr.peripheral.GPIOF),
                                 dp.GPIOG.split(ccdr.peripheral.GPIOG),
                                 dp.GPIOH.split(ccdr.peripheral.GPIOH));
    let mut delay = Delay::new(cp.SYST, ccdr.clocks);
    let mut led_user = daisy::led::LedUser::new(pins.LED_USER);

    // Initialize the heap allocator
    // let start = cortex_m_rt::heap_start() as usize;
    // let size = 1_048_576; // in bytes
    // let size = 512_000; // in bytes
    // unsafe { ALLOCATOR.init(start, size) }

    let i2c2_pins = (
        pins.WM8731.SCL.into_alternate_af4(),
        pins.WM8731.SDA.into_alternate_af4(),
    );

    let sai1_pins = (
        pins.WM8731.MCLK_A.into_alternate_af6(),
        pins.WM8731.SCK_A.into_alternate_af6(),
        pins.WM8731.FS_A.into_alternate_af6(),
        pins.WM8731.SD_A.into_alternate_af6(),
        pins.WM8731.SD_B.into_alternate_af6(),
    );

    loggit!("Setting up audio");
    let sai1_prec = ccdr
        .peripheral
        .SAI1
        .kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::PLL3_P);

    let i2c2_prec = ccdr.peripheral.I2C2;

    let audio_interface = audio::Interface::init(&ccdr.clocks,
                                                 sai1_prec,
                                                 sai1_pins,
                                                 i2c2_prec,                      // added i2c init
                                                 i2c2_pins,
                                                 ccdr.peripheral.DMA1).unwrap();


    // //set up keypad
    // loggit!("Setting up keypad");
    // let keypad = keypad_new!(BlueboxenKeys {
    //     columns: (
    //         pins.SEED_PIN_3.into_pull_up_input(),
    //         pins.SEED_PIN_4.into_pull_up_input(),
    //         pins.SEED_PIN_5.into_pull_up_input(),
    //         pins.SEED_PIN_6.into_pull_up_input(),
    //     ),
    //     rows: (
    //         pins.SEED_PIN_7.into_open_drain_output(),
    //         pins.SEED_PIN_8.into_open_drain_output(),
    //         pins.SEED_PIN_9.into_open_drain_output(),
    //         pins.SEED_PIN_10.into_open_drain_output(),
    //     ),
    // });
    // let keys = keypad.decompose();

    // let rows = [
    //     pins.SEED_PIN_1.into_pull_up_input().downgrade(),
    //     pins.SEED_PIN_2.into_pull_up_input().downgrade(),
    //     pins.SEED_PIN_3.into_pull_up_input().downgrade(),
    //     pins.SEED_PIN_4.into_pull_up_input().downgrade(),
    // ];
    // let mut cols = [
    //      pins.SEED_PIN_9.into_open_drain_output().downgrade(),
    //      pins.SEED_PIN_10.into_open_drain_output().downgrade(),
    //      pins.SEED_PIN_29.into_open_drain_output().downgrade(),
    //      pins.SEED_PIN_30.into_open_drain_output().downgrade(),
    // ];
    let mut cols = [
        pins.SEED_PIN_1.into_open_drain_output().downgrade(),
        pins.SEED_PIN_2.into_open_drain_output().downgrade(),
        pins.SEED_PIN_3.into_open_drain_output().downgrade(),
        pins.SEED_PIN_4.into_open_drain_output().downgrade(),
    ];
    let rows = [
         pins.SEED_PIN_9.into_pull_up_input().downgrade(),
         pins.SEED_PIN_10.into_pull_up_input().downgrade(),
         pins.SEED_PIN_29.into_pull_up_input().downgrade(),
         pins.SEED_PIN_30.into_pull_up_input().downgrade(),
    ];
    let col_freq = [1209.0, 1336.0, 1477.0, 1633.0];
    let row_freq = [697.0, 770.0, 852.0, 941.0];

    //setup i2c4 bus for shared use
    loggit!("Setting up i2c");

    let mut scl = pins.SEED_PIN_13.into_alternate_af6().set_open_drain();
    let mut sda = pins.SEED_PIN_14.into_alternate_af6().set_open_drain();
    let mut i2c4 = dp.I2C4.i2c(
        (scl, sda),
        400.khz(),
        ccdr.peripheral.I2C4,
        &ccdr.clocks,
    );


    let i2c4_bus: &'static _ = shared_bus::new_cortexm!(hal::i2c::I2c<daisy_bsp::pac::I2C4> =  i2c4).unwrap();
    let d1b = i2c4_bus.acquire_i2c();
    let d2b = i2c4_bus.acquire_i2c();
    let d3b = i2c4_bus.acquire_i2c();
    let mut display_1 = HT16K33::new(d1b, 0x70);
    let mut display_2 = HT16K33::new(d2b, 0x71);
    let mut display_3 = HT16K33::new(d3b, 0x72);
    unsafe {
        SHARED_DEVICE_1 = Some(display_1);
        SHARED_DEVICE_2 = Some(display_2);
        SHARED_DEVICE_3 = Some(display_3);
    }
    loggit!("Set up i2c");

    // let mut combined_display = DisplayDriver::init_display();


    // - audio callback -------------------------------------------------------
    loggit!("Setting up callback");

    // handle callback with function pointer
    // #[cfg(not(feature = "alloc"))]
    //     let audio_interface = {
    //     fn callback(fs: f32, block: &mut audio::Block) {
    //         static mut OSC_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    //         static mut OSC_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    //         unsafe { OSC_1.dx = (1. / fs) * 941.00 };
    //         unsafe { OSC_2.dx = (1. / fs) * 1477.00 };
    //         for frame in block {
    //             *frame = (unsafe { OSC_1.step() },
    //                       unsafe { OSC_2.step() });
    //         }
    //     }
    //
    //     audio_interface.spawn(callback)
    // };

    // handle callback with closure (needs alloc)
    // #[cfg(any(feature = "alloc"))]
    let audio_interface = {
        let mut osc_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
        let mut osc_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);

        audio_interface.spawn(move |fs, block| {
            cortex_m::interrupt::free(|cs| {
                let tone_a = TONE_A.borrow(cs).get();
                let tone_b = TONE_B.borrow(cs).get();
                if tone_a.is_some() && tone_b.is_some() {
                    osc_1.dx = (1. / fs) * tone_a.unwrap();
                    osc_2.dx = (1. / fs) * tone_b.unwrap();
                    for frame in block {
                        *frame = (osc_1.step(), osc_2.step());
                    }
                }
            })
        })
    };

    let audio_interface = match audio_interface {
        Ok(audio_interface) => audio_interface,
        Err(e) => {
            loggit!("Failed to start audio interface: {:?}", e);
            loop {}
        }
    };

    cortex_m::interrupt::free(|cs| {
        AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
    });


    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().0 / 1000;
    let mut ctr = 0u32;

    loop {
        ctr += 1;
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
        // combined_display.display_numbers_frame();
        // combined_display.hello_world();
        // for (row_index, row) in keys.iter().enumerate() {
        //     loggit!("row {}: ", row_index);
        //     for key in row.iter() {
        //         let is_pressed = if key.is_low() { 1 } else { 0 };
        //         loggit!(" {} ", is_pressed);
        //     }
        //     loggit!();
        // }

        let mut col_tone: Option<f32> = None;
        let mut row_tone: Option<f32> = None;
        let mut found_it = false;
        cols.iter_mut().for_each(|mut c| { c.borrow_mut().set_low(); });
        asm::delay(1000000);
        match rows.iter().enumerate().find(|(i, r)| r.is_low().unwrap()) {
            Some((row_idx, r)) => {
                cols.iter_mut().enumerate().for_each(|(col_idx, mut c)| {
                    // loggit!("checking col {} row {}", j, i);
                    c.borrow_mut().set_high();
                    asm::delay(1000000);
                    if !found_it && !r.is_low().unwrap() {
                        found_it = true;
                        row_tone = Some(row_freq[row_idx]);
                        col_tone = Some(col_freq[col_idx]);
                        // loggit!("c{} r{}", col_idx, row_idx);
                    }
                });
            }
            _ => (),
        }

        cortex_m::interrupt::free(|cs| {
            TONE_A.borrow(cs).set(col_tone);
            TONE_B.borrow(cs).set(row_tone);
        });
    }
}


// - interrupts ---------------------------------------------------------------

// interrupt handler for: dma1, stream1
#[interrupt]
fn DMA1_STR1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match audio_interface.handle_interrupt_dma1_str1() {
                Ok(()) => (),
                Err(e) => {
                    loggit!("Failed to handle interrupt: {:?}", e);
                }
            };
        }
    });
}
