//! A collection of type aliases to cut down on code clutter

use crate::stm32::{SPI1, SPI3};
use analog_multiplexer::{DummyPin, Multiplexer};
use max7219::connectors::PinConnector;
use max7219::MAX7219;
use spi_memory::series25::Flash;
use stm32f4xx_hal::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA15};
use stm32f4xx_hal::gpio::gpiob::{PB0, PB1, PB3, PB4, PB5, PB8, PB9, PB10, PB12};
use stm32f4xx_hal::gpio::gpioc::{PC13};
use stm32f4xx_hal::gpio::{AF5, AF6, Alternate, Analog, Floating, Input, Output, PullUp, PushPull};
use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use stm32f4xx_hal::spi::{NoMiso, NoSck, Spi};
use usb_device::device::UsbDevice;
use usbd_hid::hid_class::HIDClass;
use usbd_serial::SerialPort;
use crate::ws2812::Ws2812;
use infrared::receiver::{Receiver, PinInput, Poll};
use infrared::protocol::*;

// pub type UsbKeyClass = keyberon::Class<'static, UsbBusType, dyn Leds>;
pub type UsbKeyDevice = UsbDevice<'static, UsbBusType>;
pub type UsbMouseDevice = HIDClass<'static, UsbBus<USB>>;
pub type UsbSerialDevice = SerialPort<'static, UsbBus<USB>>;

// Handy type aliases to avoid a lot of long lines/typing later...
pub type AnalogPins = (
    PA0<Analog>,
    PA1<Analog>,
    PA2<Analog>,
    PA3<Analog>,
    PA4<Analog>,
);
// Define which pins go to where on your analog multiplexer's select lines
pub type S0 = PC13<Output<PushPull>>; // These just make things easier to read/reason about
pub type S1 = PB8<Output<PushPull>>; // aka "very expressive"
pub type S2 = PB12<Output<PushPull>>;
pub type S3 = PA15<Output<PushPull>>;
pub type EN = DummyPin; // NOTE: We assume the enable pin goes to GND at all times

// Power status pin (goes floating when power is connected)
pub type POWER = PB10<Input<PullUp>>;

// Relay pin
pub type RELAY1 = PB1<Output<PushPull>>;
pub type RELAY2 = PA9<Output<PushPull>>;
pub type RELAY3 = PA10<Output<PushPull>>;

// Buzzer pin
pub type BUZZER = PA8<Output<PushPull>>;

// Display
pub type MAX7219Display =
    MAX7219<PinConnector<PA8<Output<PushPull>>, PB4<Output<PushPull>>, PB3<Output<PushPull>>>>;

// TODO: Add a proper DummyPin struct somewhere we can use with EN
// NOTE: embedded_hal really needs a DummyPin feature for things like unused driver pins!
pub type SelectPins = (S0, S1, S2, S3, EN);
pub type Multiplex = Multiplexer<SelectPins>;

// Infrared receiver
pub type IRPin = PB0<Input<Floating>>;
// Couldn't get MultiReceiver working so it's disabled for now...
// pub type IrReceiver = MultiReceiver5<Rc6, Nec, NecSamsung, Rc5, NecApple, Poll, PinInput<IRPin>>;
pub type IrReceiver = Receiver<Nec, Poll, PinInput<IRPin>>;

// WS2812B-B LEDs
pub type SPIWS2812B = Ws2812<Spi<SPI3, (NoSck, NoMiso, PB5<Alternate<AF6>>)>>;
// pub type SPIWS2812B = Ws2812<Spi<SPI1, (NoSck, NoMiso, PB5<Alternate<AF5>>)>>;

// SPI Flash
pub type SPIFlash = Flash<
    Spi<
        SPI1,
        (
            PA5<Alternate<AF5>>,
            PA6<Alternate<AF5>>,
            PA7<Alternate<AF5>>,
        ),
    >,
    PB9<Output<PushPull>>,
>;
