// #![deny(unsafe_code)]
#![no_main]
#![no_std]

//! **NOTE:** To debug this firmware you need your STM32 board connected
//! to an ST-LINK debugger and [probe-run](1).  A `.cargo/config` file is
//! included and configured with `runner = "probe-run --chip STM32F401CC"`.
//!
//! This Riskey keyboard firmware is based on Keyberon.  It reads the state
//! of all channels on multiple 16-channel analog multiplexers (up to 10
//! for a total of 160 analog channels).
//!
//! You'll also need to change the pins to whatever pins you're using with
//! your own board.  The default configuration uses:
//!
//! * `PB0`: Connected to `Z` (analog output on the multilpexer)
//! * `PB5`: Connected to `EN` (enable pin)
//! * `PB12, PB13, PB14, PB15`: Connected to `S0, S1, S2, S3` (channel select pins)
//!
//! **NOTE::** You can just run `EN` to ground to keep your analog multiplexers always enabled.
//!
//! [1]: https://github.com/knurling-rs/probe-run
//!
/*
TEMP NOTES:
    * At 3.3V the OH49E-S draws 5mA and at 5V, it draws 7.8mA.
    * OH49E-S draws 561.6mA @5V   (72 sensors)
    * OH49E-S draws 360mA   @3.3V (72 sensors)
*/

// NOTE: This is how you perform a software reset:
//       cortex_m::peripheral::SCB::sys_reset();

extern crate analog_multiplexer;
extern crate panic_halt;

// use core::sync;
use core::cmp::min;
use core::fmt::Write;
use core::ops::Div;
use core::str::from_utf8;
use core::{borrow::BorrowMut, hint::spin_loop};

// Generic embedded stuff
extern crate nb;
use heapless::String;
// use alloc::string::ToString;
use numtoa::NumToA;
use cortex_m;
// use cortex_m_rt::pre_init;
// For putting the MCU in DFU mode
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::spi::MODE_0;
use rtic::app;
use rtic::Monotonic;
use rtic::cyccnt::U32Ext as _;
use rtt_target::{rtt_init, UpChannel};
use stm32f4xx_hal::adc::{config::AdcConfig, config::SampleTime, Adc};
use stm32f4xx_hal::gpio::gpiob::PB15;
use stm32f4xx_hal::{adc::config::Clock, time::MegaHertz};
// use stm32f4xx_hal::{pwm, time::MegaHertz};
// use stm32f4xx_hal::gpio::gpioc::PC13;
use stm32f4xx_hal::gpio::{Output, PushPull};
use stm32f4xx_hal::otg_fs::{UsbBusType, USB};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::spi::{NoMiso, NoSck, Spi};
use stm32f4xx_hal::stm32::ADC1;
use stm32f4xx_hal::{stm32, timer};
// use stm32::TIM11;
// use stm32f4xx_hal::interrupt::TIM4;
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass; // Needed for keyboard.poll() and mouse.poll() to work
use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;
use usbd_serial::SerialPort;

// Keyboard-specific stuff
use analog_multiplexer::{DummyPin, Multiplexer};
use keyberon::key_code::KbHidReport;
use keyberon::key_code::KeyCode;
use keyberon::layout::{Event, Layout};

// Our own stuff
mod aliases;
mod config;
mod config_structs;
mod display;
mod ir_remote;
mod layers;
mod multiplexers;
// TODO:
// mod encoder;
// mod buzzer;

// WS2812/smart-leds stuff
use crate::ws2812::Ws2812;
use smart_leds::{brightness, colors, SmartLedsWrite, RGB8};
use ws2812_spi as ws2812;

// max7219 stuff
use max7219::*;
extern crate font8x8;

// IR receiver stuff
// use infrared::{protocol::Nec, PeriodicReceiver};
use infrared::receiver::{Receiver, PinInput, Poll};
use infrared::protocol::*;

// use infrared::protocols::capture::Capture;
// TODO: Switch the IR stuff to using interrupt-driven events if possible
// TODO: Make it so that it supports all the supported infrared protocols (not just Nec)
// This is the default value for 'ir_button' that indicates, "no buttons are currently pressed":
const IR_BUTTON_NOT_PRESSED: u8 = 255;

// SPI flash stuff
use spi_memory::prelude::*;
use spi_memory::series25::Flash;
// NOTE: It's actually 128 but if we read that much on boot it hangs for a long ass time.
// Temporarily set this to 16 for now:
const MEGABITS: u32 = 32; // Flash chip size in Mbit.
const SIZE_IN_BYTES: u32 = (MEGABITS * 1024 * 1024) / 8; // Size of flash in bytes

const DISPLAY_TICK_RATE: u32 = 84_000_000 / 1000; // How many times per second display.tick() will be called

type UsbKeyClass = keyberon::Class<'static, UsbBusType, Leds>;

const MV_DIVISOR: u16 = 4; // A cheap way to deal with voltage wobble/average things out
                           // FYI: Setting the divisor to 4 should give us 80-100 "steps" of resolution per sensor
const POWER_CHECK_INTERVAL: u32 = 84_000_000 / 2; // How often to check the state of external power (every 500ms)

// This is a numpad with the Black Pill at the top so we'll use the built-in LED for Numlock:
pub struct Leds {
    num_lock: PB15<Output<PushPull>>, // NOTE: PB15 is just a temp unused pin!
}
impl keyberon::keyboard::Leds for Leds {
    fn num_lock(&mut self, status: bool) {
        if status {
            self.num_lock.set_low().unwrap()
        } else {
            self.num_lock.set_high().unwrap()
        }
    }
}

#[app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        config: config_structs::Config,
        debug_ch0: UpChannel,
        debug_ch1: UpChannel,
        debug_ch2: UpChannel,
        debug_ch3: UpChannel,
        debug_ch4: UpChannel,
        debug_ch5: UpChannel,
        debug_ch6: UpChannel,
        debug_ch7: UpChannel,
        usb_key_dev: aliases::UsbKeyDevice,
        usb_keyboard: UsbKeyClass,
        usb_mouse: aliases::UsbMouseDevice,
        // serial: aliases::UsbSerialDevice, // USB serial temporarily disabled since we're not using it yet
        multiplexer: aliases::Multiplex,
        // delay: stm32f4xx_hal::delay::Delay,
        timer: timer::Timer<stm32::TIM3>,
        led_timer: timer::Timer<stm32::TIM5>,
        // display_timer: timer::Timer<stm32::TIM4>,
        ir_timer: timer::Timer<stm32::TIM2>,
        ir_button: u8, // Anything > 254 is considered, "nothing pressed"
        // relay1: aliases::RELAY1,
        // relay2: aliases::RELAY2,
        // relay3: aliases::RELAY3,
        adc: Adc<ADC1>,
        clocks: stm32f4xx_hal::rcc::Clocks,
        analog_pins: aliases::AnalogPins,
        layout: Layout<layers::CustomActions>,
        // Keep track of the *initial* (resting) value of each sensor
        channel_states: [multiplexers::ChannelStates; config::KEYBOARD_NUM_MULTIPLEXERS + 1],
        debug_msg_counter: u16,
        // debug_buf: [u8; 20], // Only used when converting numbers to strings
        debug_buf: String<{ config::DISPLAY_BUFFER_LENGTH }>, // Only used when converting numbers to strings
        // ws: Ws2812<stm32f4xx_hal::timer::Timer<stm32f4xx_hal::stm32::TIM5>, PB5<Output<PushPull>>>,
        ws: aliases::SPIWS2812B,
        ext_power: aliases::POWER,
        // flash: aliases::SPIFlash,
        leds_max_brightness: u8, // Will be one of LEDS_MAX_BRIGHTNESS_POWERED or LEDS_MAX_BRIGHTNESS_UNPOWERED
        leds_brightness_saved: u8, // So we can restore it when toggling
        led_data: [RGB8; config::LEDS_NUM_LEDS],
        led_state: u16,
        display: display::Display<{ config::DISPLAY_BUFFER_LENGTH }>,
        display_brightness_saved: u8, // So we can restor it when toggling
        display_message: display::Message<{ config::DISPLAY_BUFFER_LENGTH }>,
        rotary_clockwise: bool,
        // ir_recv: PeriodicReceiver<Nec, aliases::IRPin>,
        ir_recv: aliases::IrReceiver,
        // buzzer: pwm::PwmChannels<stm32::TIM1, C1>,
    }

    // This idle loop fixes a bug that prevents flashing without the reset pin connected to the ST-LINK:
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            spin_loop();
        }
    }

    #[init(schedule = [power_check, update_defaults, update_display, tick_display])]
    // #[init]
    fn init(mut c: init::Context) -> init::LateResources {
        // Static stuff
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
        // let debug_buf = [0u8; 20];
        let debug_buf: String<{ config::DISPLAY_BUFFER_LENGTH }> = String::new();
        // Convert the various updates-per-second values to their cycles() equivalents
        let recalibration_refresh: u32 = 84_000_000 * config::KEYBOARD_RECALIBRATION_RATE;
        let display_refresh: u32 = 84_000_000 / config::DISPLAY_REFRESH_INTERVAL;

        // TODO: Write a macro that loads these automatically
        let keyboard_config = config_structs::KeyboardConfig {
            north_down: config::KEYBOARD_NORTH_DOWN,
            actuation_threshold: config::KEYBOARD_ACTUATION_THRESHOLD,
            release_threshold: config::KEYBOARD_RELEASE_THRESHOLD,
            ignore_below: config::KEYBOARD_IGNORE_BELOW,
            recalibration_rate: recalibration_refresh,
            num_multiplexers: config::KEYBOARD_NUM_MULTIPLEXERS,
            max_channels: config::KEYBOARD_MAX_CHANNELS,
            usb_vid: config::KEYBOARD_USB_VID,
            usb_pid: config::KEYBOARD_USB_PID,
        };
        let mouse_config = config_structs::MouseConfig {
            scroll_amount: config::MOUSE_SCROLL_AMOUNT,
        };
        let encoder_config = config_structs::EncoderConfig {
            mux: config::ENCODER_MUX,
            resolution: config::ENCODER_RESOLUTION,
            press_threshold: config::ENCODER_PRESS_THRESHOLD,
            channel1: config::ENCODER_CHANNEL1,
            channel2: config::ENCODER_CHANNEL2,
            press_channel: config::ENCODER_PRESS_CHANNEL,
        };
        let leds_config = config_structs::LedsConfig {
            brightness: config::LEDS_BRIGHTNESS,
            max_brightness_unpowered: config::LEDS_MAX_BRIGHTNESS_UNPOWERED,
            max_brightness_powered: config::LEDS_MAX_BRIGHTNESS_POWERED,
            step: config::LEDS_STEP,
            num_leds: config::LEDS_NUM_LEDS,
            speed: config::LEDS_SPEED,
        };
        let infrared_config = config_structs::InfraredConfig {
            encoding: config::INFRARED_ENCODING,
            mux: config::INFRARED_MUX,
        };
        let display_config = config_structs::DisplayConfig {
            num_matrices: config::DISPLAY_NUM_MATRICES,
            brightness: config::DISPLAY_BRIGHTNESS,
            max_brightness: config::DISPLAY_MAX_BRIGHTNESS,
            buffer_length: config::DISPLAY_BUFFER_LENGTH,
            refresh_interval: display_refresh,
            vertical_flip: config::DISPLAY_VERTICAL_FLIP,
            mirror: config::DISPLAY_MIRROR,
        };
        let dev_config = config_structs::DevConfig {
            debug_refresh_interval: config::DEV_DEBUG_REFRESH_INTERVAL,
        };
        // Load our user-configured default Config
        let config = config_structs::Config {
            keyboard: keyboard_config,
            mouse: mouse_config,
            encoder: encoder_config,
            leds: leds_config,
            display: display_config,
            infrared: infrared_config,
            dev: dev_config,
        };

        // Initialize (enable) the monotonic timer (CYCCNT)
        // c.core.DCB.enable_trace();
        c.core.DWT.enable_cycle_counter();

        // Boilerplate for setting up the board
        let rcc = c.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.mhz())
            .sysclk(84.mhz()) // Should be 84 for Riskeyboard 70
            .require_pll48clk()
            .freeze();

        // Create a delay abstraction based on SysTick
        // let delay = stm32f4xx_hal::delay::Delay::new(c.core.SYST, clocks);

        // Setup a 1000Hz timer for RTIC/USB FS polling
        let mut keys_timer = timer::Timer::tim3(c.device.TIM3, 1.khz(), clocks);
        keys_timer.listen(timer::Event::TimeOut);

        // Schedule a task that occasionally checks if the default mV values need to be adjusted
        c.schedule
            .update_defaults(c.start + config.keyboard.recalibration_rate.cycles())
            .unwrap();

        // Setup another timer for the RGB LEDs
        let mut led_timer = timer::Timer::tim5(c.device.TIM5, config.leds.speed.hz(), clocks);
        led_timer.listen(timer::Event::TimeOut);
        let led_state: u16 = 0;

        // Setup two timers for the display:
        // * One that shifts the message left according to the configured display_refresh
        // * One that calls tick() every millisecond
        c.schedule
            .update_display(c.start + display_refresh.cycles())
            .unwrap();
        // let mut display_timer = timer::Timer::tim4(c.device.TIM4, display_refresh.hz(), clocks);
        // display_timer.listen(timer::Event::TimeOut);
        c.schedule
            .tick_display(c.start + DISPLAY_TICK_RATE.cycles())
            .unwrap();

        // So we can use GPIOs...
        let gpioa = c.device.GPIOA.split();
        let gpiob = c.device.GPIOB.split();
        let gpioc = c.device.GPIOC.split();

        // TEMP: Set PC13 to (floating input, pull down/up input == flicker)
        // into_open_drain_output, into_push_pull_output == broken (no updates to LEDs)
        // let _temp_pc13 = gpioc.pc13.into_open_drain_output();

        // Power detection input (if barrel jack is connected it will show as high)
        let ext_power = gpiob.pb10.into_pull_up_input();
        // Setup a timer that checks the state of the external barrel jack
        c.schedule
            .power_check(c.start + POWER_CHECK_INTERVAL.cycles())
            .unwrap();

        // Temporarily disabled relays while I figure out how to handle them better
        // // Relay pins
        // let mut relay1 = gpiob.pb1.into_push_pull_output(); // NOTE: This is the buzzer pin
        // let mut relay2 = gpioa.pa9.into_push_pull_output();
        // let mut relay3 = gpioa.pa10.into_push_pull_output();
        // // Relay activates on low so we set high by default:
        // let _ = relay1.set_high();
        // let _ = relay2.set_high();
        // let _ = relay3.set_high();

        // Infrared LED
        let ir_sensor_pin = gpiob.pb0.into_floating_input();
        // ir_sensor_pin.make_interrupt_source(&mut c.device.SYSCFG);
        // let ir_recv: PeriodicReceiver<Nec, aliases::IRPin> =
        //     PeriodicReceiver::new(ir_sensor_pin, config::IR_SAMPLERATE);
        // let ir_recv = Receiver::new(config::IR_SAMPLERATE as usize, PinInput(ir_sensor_pin));
        // let ir_recv = Receiver::builder()
        //     .nec()
        //     .polled()
        //     .resolution(config::IR_SAMPLERATE as usize)
        //     .pin(ir_sensor_pin)
        //     .build();
        let ir_recv = Receiver::with_pin(config::IR_SAMPLERATE as usize, ir_sensor_pin);
        // Setup a 20KHz timer for IR polling
        let mut ir_timer = timer::Timer::tim2(c.device.TIM2, config::IR_SAMPLERATE.hz(), clocks);
        ir_timer.listen(timer::Event::TimeOut);
        let ir_button: u8 = IR_BUTTON_NOT_PRESSED; // Used to indicate to check_keypress() if an IR remote button was pressed

        // Buzzer stuff
        // let mut buzzer_pin = gpioa.pa8.into_push_pull_output();
        // let buzzer_pin = gpioa.pa8.into_alternate_af1();
        // let _ = buzzer_pin.set_low(); // Have to start it low
        // let channels = (
        //     gpioa.pa8.into_alternate_af1(),
        //     gpioa.pa9.into_alternate_af1(),
        // );

        // let pwm = pwm::tim1(c.device.TIM1, channels, clocks, 150.hz());
        // let (mut buzzer, _ch2) = pwm;
        // // let max_duty = buzzer.get_max_duty();
        // let duty = 1_000_000.0 / 20.0;
        // buzzer.set_duty(duty as u16 / 50);
        // // buzzer.enable();
        // buzzer.disable(); // Turn it off by default
        // let buzzer = buzzer::Buzzer::new(buzzer_pin)
        // let buzzer_timer = timer::Timer::tim1(c.device.TIM1, 1000.khz(), clocks);

        // Setup SPI flash
        let flash_cs = {
            let mut cs = gpiob.pb9.into_push_pull_output();
            cs.set_high().unwrap(); // deselect
            cs
        };
        let flash_spi = {
            let sck = gpioa.pa5.into_alternate_af5();
            let miso = gpioa.pa6.into_alternate_af5();
            let mosi = gpioa.pa7.into_alternate_af5();
            Spi::spi1(
                c.device.SPI1,
                (sck, miso, mosi),
                MODE_0,
                MegaHertz(21).into(),
                clocks,
            )
        };
        let mut flash = Flash::init(flash_spi, flash_cs).unwrap();
        let id = flash.read_jedec_id().unwrap();
        // let mut addr = 0;
        // const BUF: usize = 32;
        // let mut buf = [0; BUF];
        // while addr < SIZE_IN_BYTES {
        //     flash.read(addr, &mut buf).unwrap();
        //     addr += BUF as u32;
        // }

        // Setup MAX7219 display
        let data = gpioa
            .pa8
            .into_push_pull_output()
            .set_speed(stm32f4xx_hal::gpio::Speed::VeryHigh);
        let sck = gpiob
            .pb3
            .into_push_pull_output()
            .set_speed(stm32f4xx_hal::gpio::Speed::VeryHigh);
        let cs = gpiob
            .pb4
            .into_push_pull_output()
            .set_speed(stm32f4xx_hal::gpio::Speed::VeryHigh);
        let max7219_display =
            MAX7219::from_pins(config::DISPLAY_NUM_MATRICES, data, cs, sck).unwrap();
        let mut display = display::Display::new(max7219_display, config::DISPLAY_NUM_MATRICES);
        // TODO: Make the default display message configurable via the Conf.toml (somehow)
        let mut default_message = String::new();
        // default_message.push_str("=) TYPE TYPE TYPE =) «» ");
        default_message.push_str("Riskeyboard 70 «» ").unwrap();
        let display_message = display::Message::new(String::new(), 1);
        display.init(&default_message);

        // Setup WS2812B-B SPI output
        let mosi = gpiob
            .pb5
            .into_alternate_af6()
            .set_speed(stm32f4xx_hal::gpio::Speed::VeryHigh);
        // Configure SPI with 3Mhz rate
        let spi = Spi::spi3(
            c.device.SPI3,
            (NoSck, NoMiso, mosi),
            ws2812::MODE,
            3_000_000.hz(),
            clocks,
        );
        // Setup WS2812B-B SPI output
        // let mosi = gpiob.pb5.into_alternate_af5();
        // // Configure SPI with 3Mhz rate
        // let spi = Spi::spi1(
        //     c.device.SPI1,
        //     (NoSck, NoMiso, mosi),
        //     ws2812::MODE,
        //     3_000_000.hz(),
        //     clocks,
        // );
        let mut ws = Ws2812::new(spi);
        let leds_brightness_saved = config::LEDS_BRIGHTNESS;
        let mut leds_max_brightness = config::LEDS_MAX_BRIGHTNESS_UNPOWERED; // Start out with the low value
        let display_brightness_saved = config::DISPLAY_BRIGHTNESS;
        // Make the LEDs off for startup
        let led_data = [RGB8::default(); config::LEDS_NUM_LEDS];
        // let led_data = [colors::WHITE; config::LEDS_NUM_LEDS]; // Make em all white by default
        // for i in 0..config::LEDS_NUM_LEDS {
        //     // Make em rainbow colored to start
        //     led_data[i] = wheel((((i * 256) as u16 / config::LEDS_NUM_LEDS as u16) & 255) as u8);
        // }
        // We need the LEDs to be on at startup so we can get correct baseline sensor values
        ws.write(brightness(
            led_data.iter().cloned(),
            config::LEDS_BRIGHTNESS,
        ))
        .unwrap();
        // delay.delay_ms(5u8);
        // ws.write(brightness(led_data.iter().cloned(), config::LEDS_BRIGHTNESS)).unwrap();
        // TODO: Change this to indicate caps lock in the right place:
        // Setup our on-board LED to indicate Caps Lock
        // NOTE: Disabled the Keyberon LED control feature until I figure out how to get it working with ws2812
        let mut led = gpiob.pb15.into_push_pull_output();
        led.set_low().unwrap();
        let leds = Leds { num_lock: led };

        // Analog setup...
        // Pclk2_div_2 gives us an ADC running at 42Mhz (because sysclk is at 84Mhz)
        let adc_config = AdcConfig::default().clock(Clock::Pclk2_div_2);
        // NOTE: With a 42Mhz ADC clock and a SampleTime of Cycles_480 we'd be able
        // to read all 16 channels on all six analog multiplexers (96 pins) in about
        // 1.25ms which is just *barely* too slow for a 1000Hz USB polling rate.
        // For this reason we take the next cycle down at Cycles_28 which results in
        // a read-all-channels time of about 0.160ms. So that's the best accuracy
        // we can get without going over our 1ms budget.
        // NOTE: Formula for calcuating number of cycles: ((<cycles>+12)/42)*96
        let mut adc = Adc::adc1(c.device.ADC1, true, adc_config);
        let pa0 = gpioa.pa0.into_analog();
        let pa1 = gpioa.pa1.into_analog();
        let pa2 = gpioa.pa2.into_analog();
        let pa3 = gpioa.pa3.into_analog();
        let pa4 = gpioa.pa4.into_analog();
        let analog_pins = (pa0, pa1, pa2, pa3, pa4);
        // Setup PB12-PB15 for accessing S0-S3 on the 74HC4067 multiplexers
        let s0 = gpioc.pc13.into_push_pull_output();
        let s1 = gpiob.pb8.into_push_pull_output();
        let s2 = gpiob.pb12.into_push_pull_output();
        let s3 = gpioa.pa15.into_push_pull_output();
        let en = DummyPin; // Just run it to GND to keep always-enabled
        let select_pins = (s0, s1, s2, s3, en);

        // USB Stuff
        let usb = USB {
            // FULL SPEEEEEEEEEEEED! (aka FS)
            usb_global: c.device.OTG_FS_GLOBAL,
            usb_device: c.device.OTG_FS_DEVICE,
            usb_pwrclk: c.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
            hclk: clocks.hclk(), // stm32f4xx_hal version 0.9+ requires this
        };
        *USB_BUS = Some(UsbBusType::new(usb, EP_MEMORY));
        let usb_bus = USB_BUS.as_ref().unwrap();

        // Setup a USB serial port for controlling the keyboard from a PC
        // let serial = SerialPort::new(&usb_bus);
        // Setup a USB mouse so we can send mouse events
        let usb_mouse = HIDClass::new(&usb_bus, MouseReport::desc(), 60);
        // Setup the rest of the USB stuff
        let usb_keyboard = keyberon::new_class(usb_bus, leds);
        let usb_vid_pid = UsbVidPid(config::KEYBOARD_USB_VID, config::KEYBOARD_USB_PID);
        let usb_key_dev = UsbDeviceBuilder::new(usb_bus, usb_vid_pid)
            .manufacturer("Riskable")
            .product("Riskeyboard 70")
            // Include a capabilities string (future stuff)
            .serial_number(concat!(env!("SERIALNOW"), " 🖮v1.0:BEEF"))
            .max_power(500) // Pull out as much as we can (for now)!
            .build();

        // Only the choosiest choose these select cuts of selects
        let mut multiplexer = Multiplexer::new(select_pins);
        // Keep track of channel states/values
        let mut ch_states0: multiplexers::ChannelStates = Default::default();
        // let mut ch_states0: multiplexers::ChannelStates = Default::default();
        let mut ch_states1: multiplexers::ChannelStates = Default::default();
        let mut ch_states2: multiplexers::ChannelStates = Default::default();
        let mut ch_states3: multiplexers::ChannelStates = Default::default();
        let mut ch_states4: multiplexers::ChannelStates = Default::default();
        // This is just a pretend multiplexer for IR remote stuff:
        let ch_states5: multiplexers::ChannelStates = Default::default();
        // Setup some default values for the rotary encoder
        let rotary_clockwise: bool = true;
        // Read in the initial millivolt values for all analog channels so we have
        // a default/resting state to evaluate against.  We'll set new defaults later
        // after we've captured a few values (controlled by DEFAULT_WAIT_MS).
        for chan in 0..16 {
            // This sets the channel on all multiplexers simultaneously
            // (since they're all connected to the same S0,S1,S2,S3 pins).
            multiplexer.set_channel(chan);
            // delay.delay_us(10_u8); // Wait a moment for things to settle down between channels
            // Read the analog value of each channel/key and store it in our ChannelStates struct...
            for multi in 0..config::KEYBOARD_NUM_MULTIPLEXERS {
                // This was the best I could do for iteration rather than repetition :(
                let sample_time = SampleTime::Cycles_28;
                let sample = match multi {
                    0 => adc.convert(&analog_pins.0, sample_time),
                    1 => adc.convert(&analog_pins.1, sample_time),
                    2 => adc.convert(&analog_pins.2, sample_time),
                    3 => adc.convert(&analog_pins.3, sample_time),
                    4 => adc.convert(&analog_pins.4, sample_time),
                    _ => 0, // Riskeyboard 70 only has 5 multiplexers
                };
                let millivolts = adc.sample_to_millivolts(sample).div(MV_DIVISOR);
                match multi {
                    0 => ch_states0[chan as usize].update_default(millivolts),
                    1 => ch_states1[chan as usize].update_default(millivolts),
                    2 => ch_states2[chan as usize].update_default(millivolts),
                    3 => ch_states3[chan as usize].update_default(millivolts),
                    4 => ch_states4[chan as usize].update_default(millivolts),
                    _ => {}
                };
                // match multi {
                //     0 => ch_states0.update_default_by_index(chan as usize, millivolts),
                //     1 => ch_states1.update_default_by_index(chan as usize, millivolts),
                //     2 => ch_states2.update_default_by_index(chan as usize, millivolts),
                //     3 => ch_states3.update_default_by_index(chan as usize, millivolts),
                //     4 => ch_states4.update_default_by_index(chan as usize, millivolts),
                //     _ => {}
                // };
            }
        }
        let channel_states = [
            ch_states0, ch_states1, ch_states2, ch_states3, ch_states4, ch_states5,
        ];
        let debug_msg_counter: u16 = 0;

        // Setup some rtt-target debugging:
        let rtt_channels = rtt_init! { // NOTE: DO NOT MOVE THIS HIGHER
            up: {
                0: {
                    size: 512
                    name: "Main"
                }
                1: {
                    size: 512
                    name: "AM0"
                }
                2: {
                    size: 512
                    name: "AM1"
                }
                3: {
                    size: 512
                    name: "AM2"
                }
                4: {
                    size: 512
                    name: "AM3"
                }
                5: {
                    size: 512
                    name: "AM4"
                }
                6: {
                    size: 512
                    name: "Other"
                }
                7: {
                    size: 512
                    name: "Display"
                }
            }
        };
        let mut debug_ch0: UpChannel = rtt_channels.up.0;
        let debug_ch1: UpChannel = rtt_channels.up.1; // General messages
        let debug_ch2: UpChannel = rtt_channels.up.2; // AM0
        let debug_ch3: UpChannel = rtt_channels.up.3; // AM1
        let debug_ch4: UpChannel = rtt_channels.up.4; // AM2
        let debug_ch5: UpChannel = rtt_channels.up.5; // AM3
        let debug_ch6: UpChannel = rtt_channels.up.6; // AM4
        let debug_ch7: UpChannel = rtt_channels.up.7; // Display-specific stuff
        let _ = writeln!(debug_ch0, "init()");
        // Check the state of the external power connector
        match ext_power.is_high() {
            Ok(result) => {
                let _ = writeln!(debug_ch0, "External Barrel Jack Connected: {}", result);
                leds_max_brightness = if result {
                    config::LEDS_MAX_BRIGHTNESS_POWERED
                } else {
                    config::LEDS_MAX_BRIGHTNESS_UNPOWERED
                }
            }
            // NOTE: This only detects if a plug is inserted into the barrel jack; it doesn't detect if power is going into it.
            _ => {
                let _ = writeln!(debug_ch0, "Strange result from ext power check");
            }
        }

        let _ = writeln!(debug_ch0, "Flash Chip ID: {:?}", id);
        // let _ = writeln!(debug_ch0, "Flash Chip ADDR: {:?}", addr);
        init::LateResources {
            config,
            debug_ch0,
            debug_ch1,
            debug_ch2,
            debug_ch3,
            debug_ch4,
            debug_ch5,
            debug_ch6,
            debug_ch7,
            debug_buf,
            usb_key_dev,
            usb_keyboard,
            usb_mouse,
            // serial,
            multiplexer,
            // delay,
            timer: keys_timer,
            led_timer,
            // display_timer,
            ir_timer,
            ir_button,
            // relay1, relay2, relay3,
            adc,
            clocks,
            analog_pins,
            layout: Layout::new(layers::LAYERS),
            channel_states,
            debug_msg_counter,
            ws,
            // flash,
            ext_power,
            leds_max_brightness,
            leds_brightness_saved,
            led_data,
            led_state,
            display,
            display_brightness_saved,
            display_message,
            rotary_clockwise,
            ir_recv,
            // buzzer,
        }
    }

    // Checks the state of the power jack and updates leds_max_brightness accordingly
    #[task(priority = 1, resources = [config, leds_max_brightness, ext_power, debug_ch6], schedule = [power_check])]
    fn power_check(mut c: power_check::Context) {
        let ext_power = c.resources.ext_power;
        let mut _debug_ch6 = c.resources.debug_ch6;
        let max_brightness_powered = c
            .resources
            .config
            .lock(|conf| conf.leds.max_brightness_powered);
        let max_brightness_unpowered = c
            .resources
            .config
            .lock(|conf| conf.leds.max_brightness_unpowered);
        // let powered = match ext_power.is_high() {
        //     Ok(result) => result,
        //     _ => false, // Something went wrong; which would be wacky
        // };
        let max_brightness = c.resources.leds_max_brightness.lock(|max_brightness| {
            match ext_power.is_high() {
                Ok(result) => {
                    *max_brightness = if result {
                        max_brightness_powered
                    } else {
                        max_brightness_unpowered
                    };
                    // _debug_ch6.lock(|debug_ch6| {
                    //     let _ = writeln!(
                    //         debug_ch6,
                    //         "External Barrel Jack Connected: {}, leds_max_brightness: {}",
                    //         result, max_brightness
                    //     );
                    // });
                    *max_brightness
                }
                Err(_) => todo!(),
            }
        });
        // Make sure the active brightness reflects the current power situation (in case the user unplugged power)
        let _brightness = c.resources.config.lock(|conf| {
            conf.leds.brightness = min(max_brightness, conf.leds.brightness);
            conf.leds.brightness
        });
        // _debug_ch6.lock(|debug_ch6| {
        //     let _ = writeln!(
        //         debug_ch6,
        //         "leds_brightness: {}",
        //         _brightness
        //     );
        // });
        c.schedule
            .power_check(c.scheduled + POWER_CHECK_INTERVAL.cycles())
            .unwrap();
    }

    // For now, just scrolls whatever is in the buffer to the left and pushes it to the display
    // every time it runs
    // #[task(binds = TIM4, priority = 1, resources = [display, display_timer, debug_ch6])]
    #[task(priority = 1, resources = [config, display, display_message, debug_ch6], schedule = [update_display])]
    fn update_display(mut c: update_display::Context) {
        let display_refresh = c
            .resources
            .config
            .lock(|conf| conf.display.refresh_interval);
        let disp_message = c.resources.display_message.lock(|mess| {
            let message = mess.clone();
            // Clear out the display_message so we know it's been taken care of:
            mess.text.clear();
            message
         });
        // c.resources.debug_ch6.lock(|_debug_ch6| {
        //     let _ = writeln!(
        //         _debug_ch6,
        //         "display_text: {:?} (display_time: {:?})",
        //         disp_message.text, disp_message.time
        //     );
        // });
        c.resources.display.lock(|disp| {
            if disp_message.text.len() > 0 { // Copy it to a new display::Message
                let mut string: String<{ config::DISPLAY_BUFFER_LENGTH }> = String::new();
                let _ = string.push_str(&disp_message.text.clone());
                let message = display::Message::new(string, disp_message.time);
                disp.messages.push(message).unwrap();
            }
            disp.shift_left();
            disp.sync();
            // disp.tick();
        });
        // Clear out the display_message so we know it's been taken care of:
        // c.resources.display_message.lock(|mess| mess.text.clear());
        c.schedule
            .update_display(c.scheduled + display_refresh.cycles())
            .unwrap();
    }

    // Calls display.tick() so messages get rotated in and out properly
    #[task(priority = 1, resources = [config, display, debug_ch7], schedule = [tick_display])]
    fn tick_display(mut c: tick_display::Context) {
        let _debug_ch7 = c.resources.debug_ch7;
        c.resources.display.lock(|disp| {
            // let _ = writeln!(
            //     _debug_ch7,
            //     "display messages: {:?}",
            //     disp.messages
            // );
            match disp.tick() {
                Ok(_) => {
                    // let _ = writeln!(
                    //     _debug_ch7,
                    //     "display messages: {:?}",
                    //     disp.messages
                    // );
                },
                Err(_) => {
                    let _ = writeln!(
                        _debug_ch7,
                        "error writing: {:?}",
                        disp.messages
                    );
                },
            }
        });
        // c.resources.display.lock(|disp| {
        //     disp.tick();
        // });
        c.schedule
            .tick_display(c.scheduled + DISPLAY_TICK_RATE.cycles())
            .unwrap();
    }

    // Extra high priority on this one since it is *very* sensitive to timing
    #[task(binds = TIM2, priority = 5, resources = [ir_button, ir_timer, ir_recv, display_message, debug_buf, debug_ch6])]
    fn do_ir(c: do_ir::Context) {
        c.resources.ir_timer.clear_interrupt(timer::Event::TimeOut);
        // let mut buf = [0u8; 20];
        let ir_recv = c.resources.ir_recv;
        let _debug_ch6 = c.resources.debug_ch6;
        // let debug_buf = c.resources.debug_buf;
        // let now: = Monotonic::now();
        // let now = c.start;
        // let dt = now
        //     .checked_duration_since(&last_event)
        //     .and_then(|v| Microseconds::<u32>::try_from(v).ok())
        //     .map(|ms| ms.0 as usize)
        //     .unwrap_or_default();
        if let Ok(Some(cmd)) = ir_recv.poll() {
            use infrared::remotecontrol::RemoteControlModel;
            // use infrared::remotecontrol::RemoteControl;
            // let _ = writeln!(
            //     _debug_ch6,
            //     "IR: {:?} (ir_button: {:?})",
            //     cmd, c.resources.ir_button
            // );
            if !cmd.repeat {
                // Only do this once
                if let Some(button) = ir_remote::IRRemote::decode(&cmd) {
                    let index = ir_remote::IRRemote::index(button) as u8;
                    let _ = writeln!(_debug_ch6, "{:?} (index: {})", button, index);
                    // let mut _cmd: &[u8; 1] = &[cmd.cmd];
                    let (code, _) = ir_remote::IRRemote::BUTTONS[index as usize];
                    let mut string: String<{ config::DISPLAY_BUFFER_LENGTH }> = String::new();
                    let mut buf = [0u8; 20];
                    // let _cmd = ir_remote::IRRemote::to_string(code);
                    // let _cmd = code.numtoa_str(10, &mut NUM_BUFFER);
                    let _ = string.push_str(code.numtoa_str(10, &mut buf));
                    // let s_slice: &str = &string;
                    // let s_slice: &str = debug_buf.as_str();
                    // let _cmd = code.numtoa_str(10, debug_buf);
                    // let _cmd = core::str::from_utf8(&[cmd.cmd]).unwrap();
                    // let _cmd: &[u8; 1] = &[_cmd as u8];
                    // let _cmd = core::str::from_utf8(&[code as u8]).unwrap();
                    // let _cmd = cmd.cmd.to_string();
                    // *c.resources.display_message = display::Message::new("It works    ", 5000);
                    // *c.resources.display_message = display::Message::new(string, 3000);
                    *c.resources.ir_button = index; // This is how tick() can know what was pressed
                }
                // TODO: Make this preserve the default message and restore it after a time
                // c.resources.display.lock(|disp| {
                //     // Display the IR code
                //     disp.write_str(c.resources.ir_button);
                // });
                // c.resources.display.write_str(core::str::from_utf8(&[8]).unwrap());
                // c.resources.display.write_str("Well this part works   ");
                // c.resources
                //     .display
                //     .write_str(core::str::from_utf8(&[*c.resources.ir_button]).unwrap());
            }
        }
    }

    #[task(binds = OTG_FS, priority = 3, resources = [usb_key_dev, usb_keyboard, usb_mouse])]
    fn usb_tx(mut c: usb_tx::Context) {
        usb_poll(
            &mut c.resources.usb_key_dev,
            &mut c.resources.usb_keyboard,
            &mut c.resources.usb_mouse,
            // &mut c.resources.serial,
        );
    }

    #[task(binds = OTG_FS_WKUP, priority = 3, resources = [usb_key_dev, usb_keyboard, usb_mouse])]
    fn usb_rx(mut c: usb_rx::Context) {
        usb_poll(
            &mut c.resources.usb_key_dev,
            &mut c.resources.usb_keyboard,
            &mut c.resources.usb_mouse,
            // &mut c.resources.serial,
        );
    }

    // NOTE: This is the "schedule" way of updating the LEDs (instead of using a timer)
    // Updates the state of the LEDs according to the config.led.speed setting
    // #[task(priority = 4, resources = [config, ws, led_data, led_state, debug_ch6], schedule = [update_leds])]
    // fn update_leds(mut c: update_leds::Context) {
    //     let ws = c.resources.ws;
    //     let led_data = c.resources.led_data;
    //     let speed = c.resources.config.leds.speed;
    //     let led_state = c.resources.led_state;
    //     let led_brightness = c.resources.config.leds.brightness;
    //     // let _ = c.resources.debug_ch6.lock(|chan| writeln!(chan, "update_leds()"));
    //     // Do the rainbow run thing (for now)
    //     for i in 0..config::LEDS_NUM_LEDS {
    //         led_data[i] = wheel(
    //             ((i * 256) as u16 / config::LEDS_NUM_LEDS as u16 + *led_state as u16 & 255) as u8,
    //         );
    //     }
    //     match ws.write(brightness(led_data.iter().cloned(), led_brightness)) {
    //         Ok(_) => {}
    //         Err(err) => {
    //             c.resources.debug_ch6.lock(|debug_ch6| {
    //                 let _ = writeln!(debug_ch6, "Error writing to LEDs: {:?}", err);
    //             });
    //         }
    //     }
    //     c.schedule
    //         .update_leds(c.scheduled + speed.cycles())
    //         .unwrap();
    // }

    #[task(binds = TIM5, priority = 4, resources = [config, ws, led_data, led_state, led_timer, debug_ch6])]
    // #[task(priority = 1, resources = [ws, led_data], schedule = [do_leds])]
    fn do_leds(mut c: do_leds::Context) {
        // c.resources.debug_ch6.lock(|debug_ch6| {
        //     let _ = writeln!(debug_ch6, "do_leds()");
        // });
        c.resources.led_timer.clear_interrupt(timer::Event::TimeOut);
        let led_brightness = c.resources.config.leds.brightness;
        let ws = c.resources.ws;
        let led_data = c.resources.led_data;
        let led_state = c.resources.led_state;
        // TODO: Make the effect configurable
        // Do the rainbow run thing (for now)
        for i in 0..config::LEDS_NUM_LEDS {
            led_data[i] = wheel(
                ((i * 256) as u16 / config::LEDS_NUM_LEDS as u16 + *led_state as u16 & 255) as u8,
            );
        }
        match ws.write(brightness(led_data.iter().cloned(), led_brightness)) {
            Ok(_) => {}
            Err(err) => {
                c.resources.debug_ch6.lock(|debug_ch6| {
                    let _ = writeln!(debug_ch6, "Error writing to LEDs: {:?}", err);
                });
            }
        }
        // Increment the LED state value
        *led_state = led_state.saturating_add(1);
        if *led_state as u16 > 65534 {
            *led_state = 0;
        }
    }

    // TODO: Make the tick() function much more simple like keyberon-f4:
    // #[task(binds = TIM3, priority = 1, resources = [usb_class, matrix, debouncer, layout, timer])]
    // fn tick(mut c: tick::Context) {
    //     c.resources.timer.clear_interrupt(timer::Event::TimeOut);
    //     for event in c
    //         .resources
    //         .debouncer
    //         .events(c.resources.matrix.get().unwrap())
    //     {
    //         c.resources.layout.event(event);
    //     }
    //     match c.resources.layout.tick() {
    //         keyberon::layout::CustomEvent::Release(()) => cortex_m::peripheral::SCB::sys_reset(),
    //         _ => (),
    //     }
    //     send_report(c.resources.layout.keycodes(), &mut c.resources.usb_class);
    // }

    // Changes the default mV values on all channels if there's a significant change
    #[task(priority = 1, resources = [config, channel_states, debug_ch0], schedule = [update_defaults])]
    fn update_defaults(mut c: update_defaults::Context) {
        let mut ch_states = c.resources.channel_states;
        let recalibration_refresh = c
            .resources
            .config
            .lock(|conf| conf.keyboard.recalibration_rate);
        // c.resources.debug_ch0.lock(|ch| {
        //     let _ = writeln!(ch, "update_defaults()");
        // });
        // First check if the user removed the top plate by checking if *all* the states changed by a large amount at once
        let mut total_difference: u32 = 0;
        for chan in 0..16 {
            for multi in 0..config::KEYBOARD_NUM_MULTIPLEXERS {
                // We need to skip the encoder since it uses the default value in a different way
                if multi == config::ENCODER_MUX {
                    if chan == config::ENCODER_CHANNEL1 || chan == config::ENCODER_CHANNEL2 {
                        break;
                    }
                }
                let ch_state = ch_states.lock(|ch_s| ch_s[multi][chan]);
                // Check if the overall voltage changed and adjust default if necessary
                if !ch_state.pressed {
                    let difference = if ch_state.value < ch_state.default {
                        ch_state.default - ch_state.value // North side down switches result in a mV drop
                    } else {
                        ch_state.value - ch_state.default // South side down switches result in a mV increase
                    };
                    total_difference = total_difference + difference as u32;
                }
            }
        }
        let max_allowed_difference = config::KEYBOARD_NUM_MULTIPLEXERS * 16 * 5;
        c.resources.debug_ch0.lock(|ch| {
            // let _ = writeln!(ch, "Total mV difference:{:?}", total_difference);
            if total_difference > max_allowed_difference as u32 {
                let _ = writeln!(
                    ch,
                    "USER REMOVED TOP PLATE (total diff: {}). Skipping recalibration...",
                    total_difference
                );
            }
        });
        if total_difference < max_allowed_difference as u32 {
            // c.resources.debug_ch0.lock(|ch| {
            //     let _ = writeln!(ch, "Checking mV differences...");
            // });
            // Now handle updating the defaults
            for chan in 0..16 {
                for multi in 0..config::KEYBOARD_NUM_MULTIPLEXERS {
                    // We need to skip the encoder since it uses the default value in a different way
                    if multi == config::ENCODER_MUX {
                        if chan == config::ENCODER_CHANNEL1 || chan == config::ENCODER_CHANNEL2 {
                            break;
                        }
                    }
                    let ch_state = ch_states.lock(|ch_s| ch_s[multi][chan]);
                    // Check if the overall voltage changed and adjust default if necessary
                    if !ch_state.pressed {
                        // NOTE: For recalibration we don't care about north/south so much as we just want to get a baseline value
                        let difference = if ch_state.value < ch_state.default {
                            ch_state.default - ch_state.value // North side down switches result in a mV drop
                        } else {
                            ch_state.value - ch_state.default // South side down switches result in a mV increase
                        };
                        // This uses the KEYBOARD_RELEASE_THRESHOLD because it should be a convenient, reasonably low value:
                        if difference > config::KEYBOARD_RELEASE_THRESHOLD {
                            c.resources.debug_ch0.lock(|ch| {
                                let _ = writeln!(
                                    ch,
                                    "New default value for mux:{:?} chan:{:?} ({:?})",
                                    multi, chan, ch_state.value
                                );
                            });
                            ch_states.lock(|states| {
                                states[multi].update_default_by_index(chan, ch_state.value)
                            });
                        }
                    }
                }
            }
        }
        c.schedule
            .update_defaults(c.scheduled + recalibration_refresh.cycles())
            .unwrap();
    }

    #[task(binds = TIM3, priority = 2, resources = [
        config, debug_msg_counter, usb_keyboard, usb_mouse, layout, multiplexer,
        leds_max_brightness, leds_brightness_saved, display_brightness_saved,
        rotary_clockwise, ir_button, display,
        // relay1, relay2, relay3,
        debug_ch0, debug_ch1, debug_ch2, debug_ch3, debug_ch4, debug_ch5,
        timer, adc, analog_pins, channel_states])]
    fn tick(mut c: tick::Context) {
        c.resources.timer.clear_interrupt(timer::Event::TimeOut);
        // Shortcuts:
        let debug_ch0 = c.resources.debug_ch0;
        let actuation_threshold = c
            .resources
            .config
            .lock(|config| config.keyboard.actuation_threshold);
        let release_threshold = c
            .resources
            .config
            .lock(|config| config.keyboard.release_threshold);
        let encoder_press_threshold = c
            .resources
            .config
            .lock(|config| config.encoder.press_threshold);
        let adc = c.resources.adc;
        let multiplexer = c.resources.multiplexer;
        let analog_pins = c.resources.analog_pins;
        let layout = c.resources.layout;
        let ch_states = c.resources.channel_states;
        let debug_msg_counter = c.resources.debug_msg_counter;
        let rotary_clockwise = c.resources.rotary_clockwise;
        let leds_brightness_saved = c.resources.leds_brightness_saved;
        let leds_max_brightness = c.resources.leds_max_brightness;
        let display_brightness_saved = c.resources.display_brightness_saved;
        let mut ir_button = c.resources.ir_button;
        let mut display = c.resources.display;
        // let display = c.resources.display.lock(|disp| {
        //     disp
        // });
        // let relay1 = c.resources.relay1;
        // let relay2 = c.resources.relay2;
        // let relay3 = c.resources.relay3;
        // let delay = c.resources.delay;
        // let buzzer = c.resources.buzzer;
        // let buzzer_timer = c.resources.buzzer_timer;
        // let _ = relay1.set_high(); // Reset it
        // FYI: I tested with all the different SampleTime options and they all provided consistent
        // results.  I chose Cycles_28 because it seemed like a safe middle ground.
        let sample_time = SampleTime::Cycles_28;
        // Loop over one channel at a time while reading said channel from all multiplexers at once
        for chan in 0..16 {
            // This sets the channel on all multiplexers simultaneously
            // (since they're all connected to the same S0,S1,S2,S3 pins).
            multiplexer.set_channel(chan);
            // delay.delay_us(15_u8); // Wait a moment for things to settle down between channels
            // Read the analog value of each channel/key and trigger a Keyberon event
            // if it's over the ACTUATION_THRESHOLD
            for multi in 0..config::KEYBOARD_NUM_MULTIPLEXERS {
                // This was the best I could do for iteration rather than repetition :(
                let sample = match multi {
                    0 => adc.convert(&analog_pins.0, sample_time),
                    1 => adc.convert(&analog_pins.1, sample_time),
                    2 => adc.convert(&analog_pins.2, sample_time),
                    3 => adc.convert(&analog_pins.3, sample_time),
                    4 => adc.convert(&analog_pins.4, sample_time),
                    _ => 0, // Riskeyboard 70 only has 5 multiplexers
                };
                let millivolts = adc.sample_to_millivolts(sample).div(MV_DIVISOR);
                // Record the channel's previous state so we can tell if it's state changed (for relays)
                let prev_ch_state = ch_states[multi][chan as usize].pressed;
                // Now record the state of each channel:
                ch_states[multi][chan as usize].record_value(millivolts);
                let _ = multiplexers::check_channel(
                    multi as usize,
                    chan as usize,
                    millivolts,
                    ch_states,
                    layout,
                    rotary_clockwise,
                    actuation_threshold,
                    encoder_press_threshold,
                    release_threshold,
                    debug_ch0,
                );
                // if ch_states[multi][chan as usize].pressed {
                //     let _ = writeln!(debug_ch0, "Something's pressed");
                // }
                // NOTE: Temporarily disabled relay support while I figure out a better way to support them (without using so many pins)
                // if ch_states[multi][chan as usize].pressed != prev_ch_state {
                //     if relay1.is_high().unwrap() && relay2.is_high().unwrap() && relay3.is_high().unwrap() {
                //         let _ = relay1.set_low(); // Kick things off
                //         continue;
                //     }
                //     if relay1.is_low().unwrap() {
                //         let _ = relay1.set_high();
                //         let _ = relay2.set_low();
                //         let _ = relay3.set_high();
                //         continue;
                //     }
                //     if relay2.is_low().unwrap() {
                //         let _ = relay1.set_high();
                //         let _ = relay2.set_high();
                //         let _ = relay3.set_low();
                //         continue;
                //     }
                //     if relay3.is_low().unwrap() {
                //         let _ = relay1.set_low();
                //         let _ = relay2.set_high();
                //         let _ = relay3.set_high();
                //         continue;
                //     }
                // }
            }
        }
        // TODO: Figure out a better way to do this so we're not wasting a
        //       zillion array slots/lots of memory just to support more buttons.
        // Handle IR Remote stuff
        let button = ir_button.lock(|button| *button);
        if button < IR_BUTTON_NOT_PRESSED {
            let pressed = ch_states[config::INFRARED_MUX][button as usize].pressed;
            let _ = writeln!(debug_ch0, "{:?} pressed: {:?}", button, pressed);
            // Emulate a keypress on our imaginary multiplexer so the user can assign stuff like normal
            if !pressed {
                ch_states[config::INFRARED_MUX].press(button as usize);
                let _ = layout.event(Event::Press(config::INFRARED_MUX as u8, button));
            } else {
                // Reset the state immediately in the next call to tick()
                ch_states[config::INFRARED_MUX].release(button as usize);
                let _ = layout.event(Event::Release(config::INFRARED_MUX as u8, button));
                ir_button.lock(|button| *button = IR_BUTTON_NOT_PRESSED); // Reset it
            }
        }
        // Check if any keys are pressed
        let mut keydown = false;
        for multi in 0..config::KEYBOARD_NUM_MULTIPLEXERS + 1 {
            if ch_states[multi].pressed > 0 {
                keydown = true;
                break;
            }
        }
        // Turn off all relays if no keys are pressed (TEMP DISABLED)
        // if !keydown {
        //     let _ = relay1.set_high();
        //     let _ = relay2.set_high();
        //     let _ = relay3.set_high();
        // }
        // Do the buzzer thing:
        // if keydown {
        //     // rprintln!("Playing buzzer!");
        //     buzzer.disable();
        //     buzzer.enable();
        // } else {
        //     // rprintln!("Disabling buzzer");
        //     buzzer.disable();
        // }
        // Only post these debug messages as fast as DEBUG_REFRESH_INTERVAL because if we don't
        // they'll come in every 1ms and can you say, "insane screen flicker"? Cuz I can!
        if *debug_msg_counter > config::DEV_DEBUG_REFRESH_INTERVAL {
            // rprintln!("ChannelValues size: {:?}", core::mem::size_of::<multiplexers::ChannelValues>());
            let _ = writeln!(c.resources.debug_ch1, "{}", ch_states[0]); // Debug AM0
            let _ = writeln!(c.resources.debug_ch2, "{}", ch_states[1]); // Debug AM1
            let _ = writeln!(c.resources.debug_ch3, "{}", ch_states[2]); // Debug AM2
            let _ = writeln!(c.resources.debug_ch4, "{}", ch_states[3]); // Debug AM3
            let _ = writeln!(c.resources.debug_ch5, "{}", ch_states[4]); // Debug AM4
            *debug_msg_counter = 0;
        } else {
            *debug_msg_counter = debug_msg_counter.saturating_add(1);
        }
        // For debugging Keyberon itself (requires changing lots of stuff in Keyberon):
        // if layout.states.len() > 0 {
        //     let _ = writeln!(c.resources.debug_ch0, "main.rs layout: {}", layout);
        // }
        // if layout.sequenced.len() > 0 {
        //     let _ = writeln!(c.resources.debug_ch0, "main.rs sequenced: {:?}", layout.sequenced);
        // }
        let mut mouse_report = MouseReport {
            x: 0,
            y: 0,
            buttons: 0,
            wheel: 0,
        };
        let mut mouse_report_changed = false;
        c.resources.config.lock(|conf| {
            match layout.tick() {
                // Handle all our custom events before dealing with the normal stuff
                keyberon::layout::CustomEvent::Press(event) => {
                    let _ = writeln!(debug_ch0, "CustomEvent Press: {:?}", event);
                    match event {
                        layers::CustomActions::LEDUp => {
                            conf.leds.brightness = min(
                                conf.leds.brightness.saturating_add(config::LEDS_STEP),
                                *leds_max_brightness,
                            );
                            let _ = writeln!(debug_ch0, "LED Brightness: {}", conf.leds.brightness);
                        }
                        layers::CustomActions::LEDDown => {
                            conf.leds.brightness =
                                conf.leds.brightness.saturating_sub(config::LEDS_STEP);
                            let _ = writeln!(debug_ch0, "LED Brightness: {}", conf.leds.brightness);
                        }
                        layers::CustomActions::LEDToggle => {
                            if conf.leds.brightness > 0 {
                                *leds_brightness_saved = conf.leds.brightness;
                                conf.leds.brightness = 0;
                            } else {
                                conf.leds.brightness = *leds_brightness_saved;
                            }
                            let _ = writeln!(
                                debug_ch0,
                                "LED Toggle: LED brightness {}",
                                conf.leds.brightness
                            );
                        }
                        layers::CustomActions::DisplayUp => {
                            conf.display.brightness =
                                conf.display.brightness.saturating_add(config::LEDS_STEP);
                            // c.resources.display.lock(|disp| {
                            //     disp.set_brightness(conf.display.brightness);
                            // });
                            // display.lock(|disp| disp.set_brightness(conf.display.brightness));
                            display.set_brightness(conf.display.brightness);
                            let _ = writeln!(
                                debug_ch0,
                                "Display Brightness: {}",
                                conf.display.brightness
                            );
                        }
                        layers::CustomActions::DisplayDown => {
                            conf.display.brightness =
                                conf.display.brightness.saturating_sub(config::LEDS_STEP);
                            // c.resources.display.lock(|disp| {
                            //     disp.set_brightness(conf.display.brightness);
                            // });
                            // display.set_brightness(conf.display.brightness);
                            // display.lock(|disp| disp.set_brightness(conf.display.brightness));
                            display.set_brightness(conf.display.brightness);
                            let _ = writeln!(
                                debug_ch0,
                                "Display Brightness: {}",
                                conf.display.brightness
                            );
                        }
                        layers::CustomActions::DisplayToggle => {
                            if conf.display.brightness > 0 {
                                *display_brightness_saved = conf.display.brightness;
                                conf.display.brightness = 0;
                            } else {
                                conf.display.brightness = *display_brightness_saved;
                            }
                            // c.resources.display.lock(|disp| {
                            //     disp.set_brightness(conf.display.brightness);
                            // });
                            display.set_brightness(conf.display.brightness);
                            // display.lock(|disp| disp.set_brightness(conf.display.brightness));
                            let _ = writeln!(
                                debug_ch0,
                                "Lighting Toggle: Display Brightness: {}",
                                conf.display.brightness
                            );
                        }
                        layers::CustomActions::LightingToggle => {
                            if conf.leds.brightness > 0 {
                                *leds_brightness_saved = conf.leds.brightness;
                                conf.leds.brightness = 0;
                            } else {
                                conf.leds.brightness = *leds_brightness_saved;
                            }
                            if conf.display.brightness > 0 {
                                *display_brightness_saved = conf.display.brightness;
                                conf.display.brightness = 0;
                            } else {
                                conf.display.brightness = *display_brightness_saved;
                            }
                            // c.resources.display.lock(|disp| {
                            //     disp.set_brightness(conf.display.brightness);
                            // });
                            display.set_brightness(conf.display.brightness);
                            // display.lock(|disp| disp.set_brightness(conf.display.brightness));
                            let _ = writeln!(
                                debug_ch0,
                                "Lighting Toggle: LED brightness {}, Display Brightness: {}",
                                conf.leds.brightness, conf.display.brightness
                            );
                        }
                        layers::CustomActions::Mouse1 => {
                            mouse_report.buttons = 1 << 0;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse2 => {
                            mouse_report.buttons = 1 << 1;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse3 => {
                            mouse_report.buttons = 1 << 2;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse4 => {
                            mouse_report.wheel = conf.mouse.scroll_amount as i8;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse5 => {
                            mouse_report.wheel = -(conf.mouse.scroll_amount as i8);
                            mouse_report_changed = true;
                        }
                        _ => (),
                    }
                }
                keyberon::layout::CustomEvent::Release(event) => {
                    let _ = writeln!(debug_ch0, "CustomEvent Release: {:?}", event);
                    match event {
                        layers::CustomActions::Mouse1 => {
                            mouse_report.buttons = 0;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse2 => {
                            mouse_report.buttons = 0;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse3 => {
                            mouse_report.buttons = 0;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse4 => {
                            mouse_report.buttons = 0;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Mouse5 => {
                            mouse_report.buttons = 0;
                            mouse_report_changed = true;
                        }
                        layers::CustomActions::Reset => {
                            let _ = writeln!(debug_ch0, "Reseting MCU...");
                            cortex_m::peripheral::SCB::sys_reset();
                        }
                        _ => (),
                    }
                }
                _ => (), // Regular Keyberon keypress stuff
            }
        });
        send_keyboard_report(layout.keycodes(), &mut c.resources.usb_keyboard);
        if mouse_report_changed {
            // if relay1.is_high().unwrap() && relay2.is_high().unwrap() && relay3.is_high().unwrap() {
            //     let _ = relay1.set_low(); // Kick things off
            // } else if relay1.is_low().unwrap() {
            //     let _ = relay1.set_high();
            //     let _ = relay2.set_low();
            //     let _ = relay3.set_high();
            // } else if relay2.is_low().unwrap() {
            //     let _ = relay1.set_high();
            //     let _ = relay2.set_high();
            //     let _ = relay3.set_low();
            // } else if relay3.is_low().unwrap() {
            //     let _ = relay1.set_low();
            //     let _ = relay2.set_high();
            //     let _ = relay3.set_high();
            // }
            send_mouse_report(mouse_report, &mut c.resources.usb_mouse);
        }
    }

    // This is needed for the schedule feature of RTIC to work:
    extern "C" {
        fn EXTI0();
        fn EXTI1();
        // fn TIM1();
    }
};

fn send_keyboard_report(
    iter: impl Iterator<Item = KeyCode>,
    usb_keyboard: &mut resources::usb_keyboard<'_>,
) {
    use rtic::Mutex;
    let report: KbHidReport = iter.collect();
    if usb_keyboard.lock(|k| k.device_mut().set_keyboard_report(report.clone())) {
        while let Ok(0) = usb_keyboard.lock(|k| k.write(report.as_bytes())) {}
    }
}

fn send_mouse_report(report: MouseReport, usb_mouse: &mut resources::usb_mouse<'_>) {
    use rtic::Mutex;
    usb_mouse.lock(|usb_hid| usb_hid.borrow_mut().push_input(&report).ok());
}

fn usb_poll(
    usb_dev: &mut aliases::UsbKeyDevice,
    keyboard: &mut UsbKeyClass,
    mouse: &mut aliases::UsbMouseDevice,
    // serial: &mut aliases::UsbSerialDevice
) {
    if usb_dev.poll(&mut [keyboard, mouse]) {
        keyboard.poll();
        mouse.poll();
        // serial.poll();
        // This was taken from the USB serial port example code:
        // let mut buf = [0u8; 64];

        // match serial.read(&mut buf) {
        //     Ok(count) if count > 0 => {
        //         // Convert send char to upper case
        //         for c in buf[0..count].iter_mut() {
        //             if 0x61 <= *c && *c <= 0x7a {
        //                 *c &= !0x20;
        //                 // rprintln!("{}", c);
        //             }
        //         }
        //         // Echo back
        //         let mut write_offset = 0;
        //         while write_offset < count {
        //             match serial.write(&buf[write_offset..count]) {
        //                 Ok(len) if len > 0 => {
        //                     write_offset += len;
        //                 }
        //                 _ => {}
        //             }
        //         }
        //     }
        //     _ => {}
        // }
    }
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

// TODO: Get this working...
// This lets us put the MCU in DFU bootloader mode so you can flash without an ST-LINK:
// NOTE: This is much more reliable when you enable `inline-asm` in cortex-m
// #[pre_init]
// unsafe fn before_main() {
//     extern "C" {
//         static mut _panic_dump_start: u8;
//     }

//     use cortex_m::register::msp;

//     let start_ptr = &mut _panic_dump_start as *mut u8;

//     // Panic-persist sets a flag to the start of the dump region
//     // when a panic occurs
//     if 0x0FACADE0 == core::ptr::read_unaligned(start_ptr.cast::<usize>()) {
//         // Clear the flag
//         start_ptr.cast::<usize>().write_unaligned(0x00000000);

//         // The DFU bootloader's reset vector and initial stack pointer
//         const SYSMEM_MSP: u32 = 0x1fff0000;
//         const SYSMEM_RESET: u32 = 0x1fff0004;

//         let dfu_msp = core::ptr::read(SYSMEM_MSP as *const u32);
//         let putter: *const fn() = SYSMEM_RESET as *const fn();

//         msp::write(dfu_msp);
//         (*putter)();
//     }
// }
/*
jamesmunns
what happens is:

* a panic happens
* panic-persist writes the panic message to uninit RAM, and sets a magic word flag
* reboot happens
* on pre-init, if that flag is set, immediately change the SP and jump to the bootloader address

For some reason, that didn't always work on the F405 (or at least it was much more reliable on the stm32f072), which might be because this would be better to do in a naked assembly function.
but yeah, adamgreig is right, there's an app note that gives you the values you need for the stack pointer and jump-to location
You might also consider resetting the vector table and stack pointer back to the "right" location if you don't jump to bootloader. I'm not sure how "clean" the bootloader leaves the operating environment
(but then definitely do that in ASM, not Rust)

ALTERNATIVE NOTE from: adamgreig (@adamgreig:matrix.org):

this sequence of "write MSP, jump to function" is common enough that it's now part of cortex-m, but not yet released, so one day it will be as easy as cortex_m::asm::bootload(0 as *const u32)

you'll still need to either write VTOR to the chip-specific address, or set a chip-specific RAM remap, but the operation "read SP and RV from vector table, write SP to MSP, jump to RV" will be done for you
(write VTOR on cortex-m3 and above, set a RAM remap on cortex-m0 which don't have VTOR)
*/
