/// Configuration structs; separate from config.rs so we can use them in build.rs without having to duplicate them

// use core::fmt::Write;
// use heapless::String;
// use heapless::consts::U32;
use serde::{Serialize, Deserialize};
use num_format::{Buffer, CustomFormat};
// use core::fmt::Display;

// This lets us generate a userconfig.rs from inside build.rs that gets include!(concat!()) inside config.rs:
macro_rules! add_const_gen {
    (
        $(#[$meta:meta])*
        pub struct $struct_name:ident {
        $(
            $(#[$field_meta:meta])*
            $field_vis:vis $field_name:ident : $field_type:ty
        ),*$(,)+
        }
    ) => {
        $(#[$meta])*
        pub struct $struct_name {
            $(
                $(#[$field_meta])*
                pub $field_name : $field_type,
            )*
        }

        impl $struct_name {
            #[allow(dead_code)]
            fn field_names() -> &'static [&'static str] {
                static NAMES: &'static [&'static str] = &[$(stringify!($field_name)),*];
                NAMES
            }

            #[allow(dead_code)]
            fn gen_meta_tuple(&self, field: &'static str) -> (&str, &str, &str, Buffer) {
                let rust_format = CustomFormat::builder()
                    .separator("_")
                    .build().unwrap();
                match field {
                    $(stringify!($field_name) => {
                        let mut buf = Buffer::default();
                        buf.write_formatted(&self.$field_name, &rust_format);
                        (
                            stringify!($struct_name),
                            stringify!($field_name),
                            stringify!($field_type),
                            buf
                        )
                    }),*
                    _ => ("","","",Buffer::default())
                }
            }

            // #[allow(dead_code)]
            // fn gen_meta_tuple(&self, field: &'static str) -> (&str, &str, &str, &str) {
            //     // let rust_format = CustomFormat::builder()
            //     //     .separator("_")
            //     //     .build().unwrap();
            //     match field {
            //         $(stringify!($field_name) => {
            //             // let mut buf = Buffer::default();
            //             // buf.write_formatted(&self.$field_name, &rust_format);
            //             let mut data = String::<U32>::from("");
            //             (
            //                 stringify!($struct_name),
            //                 stringify!($field_name),
            //                 stringify!($field_type),
            //                 write!(data, "{}", &self.$field_name),
            //             )
            //         }),*
            //         _ => ("","","","")
            //     }
            // }
        }
    }
}

struct IsInt<'__, T>(&'__ T);
impl<Int : ::num_format::ToFormattedStr> IsInt<'_, Int> {
    fn pretty_display (self: Self)
      -> impl 'static + ::core::fmt::Display
    {
        let ref rust_format =
            CustomFormat::builder()
                .separator("_")
                .build()
                .unwrap()
        ;
        let mut buf = Buffer::default();
        buf.write_formatted(self.0, rust_format);
        buf
    }
}

trait Fallback<'__, T> {
    fn pretty_display (self: Self)
      -> &'__ T
    ;
}
impl<'lt, T : ::core::fmt::Display> Fallback<'lt, T>
    for IsInt<'lt, T>
{
    fn pretty_display (self: Self)
      -> &'lt T
    {
        self.0
    }
}

add_const_gen!{
/// Configuration items related to keys, sensors, and the rotary encoder
#[derive(Debug, Serialize, Deserialize)]
pub struct KeyboardConfig {
    /// Millivolts difference where we consider it a Press()
    pub actuation_threshold: u16,
    /// Millivolts above the actuation threshold where we consider it a Release() (prevents bouncing)
    pub release_threshold: u16,
    /// Millivolt values below this value will be ignored (so we can skip mux pins connected to ground)
    pub ignore_below: u16,
    /// How often to check to see if the default mV values need to be adjusted (cycles)
    pub update_defaults_rate: u32,
    /// Total number of multiplexers on this keyboard
    pub num_multiplexers: usize,
    /// Maximum number of channels per multiplexer or buttons per IR remote (use whatever is greater)
    pub max_channels: usize,
    /// The USB VID the keyboard will identify itself with
    pub usb_vid: u16,
    /// The USB PID the keyboard will identify itself with
    pub usb_pid: u16,
}
}

add_const_gen!{
/// Configuration items related to mouse emulation
#[derive(Debug, Serialize, Deserialize)]
pub struct MouseConfig {
    /// Amount of units (aka lines) to scroll per press/rotary encoder movement
    pub scroll_amount: u8,
}
}

add_const_gen!{
/// Configuration items related to a rotary encoder
#[derive(Debug, Serialize, Deserialize)]
pub struct EncoderConfig {
    /// The multiplexer the encoder sensors are connected to
    pub mux: usize,
    /// Millivolts difference required before we consider the encoder moved in either direction
    pub resolution: u16,
    /// Millivolts difference before we consider it an encoder Press()
    pub press_threshold: u16,
    /// Channel on the multiplexer where the encoder's first sensor resides
    pub channel1: usize,
    /// Channel on the multiplexer where the encoder's second sensor resides
    pub channel2: usize,
    /// Virtual channel on the multiplexer where Press() events get defined
    pub press_channel: usize,
}
}

add_const_gen!{
/// Configuration items related to the WS2812B-B RGB LEDs
#[derive(Debug, Serialize, Deserialize)]
pub struct LedsConfig {
    /// Default brightness of the LEDs (0-255)
    pub brightness: u8,
    /// Amount to change brightness when increasing or decreasing it
    pub step: u8,
    /// Total number of LEDs on or attached to the keyboard (70 plus however many you've connected)
    pub num_leds: usize,
    /// Rate (in hz) at which the LEDs are updated
    pub speed: u32,
}
}

// TODO: Make this more display-agnostic so it can work with anything
add_const_gen!{
/// Configuration items related to the MAX7219 displays/matrices
#[derive(Debug, Serialize, Deserialize)]
pub struct DisplayConfig {
    /// Total number of MAX7219 matrices connected to the keyboard
    pub num_matrices: usize,
    /// Brightness (aka display intensity) (0-8)
    pub brightness: u8,
    /// Maximum amount of characters we'll buffer (uses up RAM so don't make it too big)
    pub buffer_length: usize,
    /// If the display is upside down (0 or 1)
    pub vertical_flip: u8,
    /// If the display needs to be mirrored (0 or 1)
    pub mirror: u8,
    /// How often (Hz) the display should be re-drawn
    pub refresh_interval: u32,
}
}

add_const_gen!{
/// Configuration items related to mouse emulation
#[derive(Debug, Serialize, Deserialize)]
pub struct InfraredConfig {
    /// Amount of units (aka lines) to scroll per press/rotary encoder movement
    pub encoding: u8,
    /// Imaginary multiplexer that holds IR remote button mappings
    pub mux: usize,
}
}

add_const_gen!{
/// Configuration items related to development stuff
#[derive(Debug, Serialize, Deserialize)]
pub struct DevConfig {
    /// Minimum amount of time to wait before sending debug messages to the debugger
    pub debug_refresh_interval: u16,
}
}

/// Central location for referencing and updating runtime settings
#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    /// Keyboard configuration items
    pub keyboard: KeyboardConfig,
    /// Mouse configuration items
    pub mouse: MouseConfig,
    /// Encoder configuration items
    pub encoder: EncoderConfig,
    /// LED configuration items
    pub leds: LedsConfig,
    /// Display (MAX7219) configuration items
    pub display: DisplayConfig,
    /// Remote control (infrared) configuration items
    pub infrared: InfraredConfig,
    /// Development configuration items (e.g. debug stuff)
    pub dev: DevConfig,
}
