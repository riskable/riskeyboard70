//! Display-related code for the MAX7219 matrices.  The plan here is for this to be temporary
//! until I (or someone else; hint hint) makes an `embedded-graphics`-compatible crate for max7219.

use font8x8::UnicodeFonts;
use heapless::{String, Vec}; // fixed capacity version of `std::Vec`
extern crate font8x8;
use crate::aliases::MAX7219Display;
use crate::config;

const MESSAGES_LENGTH: usize = 16;

/// Holds our display buffer and a `heapless::Vec` that contains active messages
pub struct Display<const BUFFER_LENGTH: usize> {
    // I love const generics!
    pub num_matrices: usize, // Max 8 right now because of the limiations of the max7219 crate
    pub buffer: Vec<[u8; 8], BUFFER_LENGTH>, // Hurray for const generics!
    display: MAX7219Display,
    pub messages: Vec<Message<{ BUFFER_LENGTH }>, MESSAGES_LENGTH>, // Holds active messages; default message will have remaining == 0
}

/// A container for text that gets moved to the display so we know when
/// to remove it.  If `remaining` is 0 the message will never time out.
#[derive(Debug, Clone)]
pub struct Message<const BUFFER_LENGTH: usize> {
    /// The ``str`` containing the message text
    pub text: String<{ BUFFER_LENGTH }>,
    /// How long it will last (ms)
    pub time: u32,
    /// Whether it's active or not
    pub active: bool,
}

impl<'a, const BUFFER_LENGTH: usize> Message<BUFFER_LENGTH> {
    pub fn new(text: String<BUFFER_LENGTH>, remaining: u32) -> Self {
        Self { text, time: remaining, active: false }
    }
}

impl<'a, const BUFFER_LENGTH: usize> Display<BUFFER_LENGTH> {
    pub fn new(display: MAX7219Display, num_matrices: usize) -> Display<BUFFER_LENGTH> {
        Display {
            num_matrices: num_matrices,
            buffer: Vec::new(),
            display: display,
            messages: Vec::new(),
        }
    }

    /// Powers on the matrices, blanks out all the displays and sets their (default) intensity.
    /// Also accepts a `&str` as the default scrolling message.
    pub fn init(&mut self, message: &String<BUFFER_LENGTH>) {
        let _ = self.display.power_on();
        for d in 0..config::DISPLAY_NUM_MATRICES {
            let _ = self.display.clear_display(d);
            self.display
                .set_intensity(d, config::DISPLAY_BRIGHTNESS)
                .unwrap();
        }
        let mut string: String<{ BUFFER_LENGTH }> = String::new();
        string.push_str(&message.clone());
        let default_message: Message<BUFFER_LENGTH> = Message::new(string, 0);
        self.messages.push(default_message).unwrap();
        self.write_string(&message.clone());
        // self.write_str("=) TYPE TYPE TYPE =) «» ");
        self.sync();
    }

    /// Clears the display by emptying the buffer and performing a `sync()`
    pub fn clear(&mut self) {
        self.buffer.clear();
        self.sync();
    }

    /// Sets the display intensity (brightness) on a scale from 0-255 to match the LED brightness
    /// values (even though MAX7219 matrices only have 8 'intensity' levels).
    /// If brightness is 0 the display will be powered off.
    pub fn set_brightness(&mut self, brightness: u8) {
        // Emulate a ceil() function:
        let mut intensity = ((brightness as u16 + 31) / 32) as u8;
        if intensity == 0 {
            let _ = self.display.power_off();
        } else {
            let _ = self.display.power_on();
        }
        intensity -= 1; // 0 is actually a valid, not-powered-off intensity value
                        // For some reason I get lots of glitches if the display intensity
                        // is greater than 6.  Not sure what the deal is but visually there's
                        // basically no difference between 6 and 8 intensity so we're limiting
                        // it to 6...
        if intensity > 6 {
            intensity = 6;
        }
        for d in 0..config::DISPLAY_NUM_MATRICES {
            self.display.set_intensity(d, intensity).unwrap();
        }
    }

    /// Writes the given *chars* to the buffer then pushes it out to the display.
    /// If there's not enough room on the display the characters will be scrolled
    /// according to the refresh interval.
    /// The buffer is emptied before writing the given *chars*.
    pub fn write_str(&mut self, chars: &str) {
        self.buffer.clear(); // Empty it out
        let error_char = font8x8::BOX_UNICODE[115].byte_array(); // ╳
        for char in chars.chars() {
            let mut data = font8x8::LATIN_FONTS
                .get(char)
                .or_else(|| font8x8::BLOCK_FONTS.get(char))
                .or_else(|| font8x8::MISC_FONTS.get(char))
                .or_else(|| font8x8::GREEK_FONTS.get(char))
                .or_else(|| font8x8::BOX_FONTS.get(char))
                .or_else(|| font8x8::SGA_FONTS.get(char))
                .or_else(|| font8x8::HIRAGANA_FONTS.get(char))
                .or_else(|| font8x8::BASIC_FONTS.get(char))
                .unwrap_or_else(|| error_char);
            if config::DISPLAY_VERTICAL_FLIP == 1 {
                data.reverse();
            }
            self.buffer.push(data).unwrap();
        }
        self.buffer.reverse();
    }

    /// Writes the given *chars* to the buffer then pushes it out to the display.
    /// If there's not enough room on the display the characters will be scrolled
    /// according to the refresh interval.
    /// The buffer is emptied before writing the given *chars*.
    pub fn write_string(&mut self, chars: &String<BUFFER_LENGTH>) -> Result<(), [u8; 8]> {
        self.buffer.clear(); // Empty it out
        let mut result = Ok(());
        let error_char = font8x8::BOX_UNICODE[115].byte_array(); // ╳
        for char in chars.chars() {
            let mut data = font8x8::LATIN_FONTS
                .get(char)
                .or_else(|| font8x8::BLOCK_FONTS.get(char))
                .or_else(|| font8x8::MISC_FONTS.get(char))
                .or_else(|| font8x8::GREEK_FONTS.get(char))
                .or_else(|| font8x8::BOX_FONTS.get(char))
                .or_else(|| font8x8::SGA_FONTS.get(char))
                .or_else(|| font8x8::HIRAGANA_FONTS.get(char))
                .or_else(|| font8x8::BASIC_FONTS.get(char))
                .unwrap_or_else(|| error_char);
            if config::DISPLAY_VERTICAL_FLIP == 1 {
                data.reverse();
            }
            // match self.buffer.push(error_char) {
            //     // Some(i) => { println!("Matched {:?}!", i); }
            //     Ok(_) => todo!(),
            //     Err(_) => todo!(),
            // }
            // self.buffer.push(error_char)
            result = self.buffer.push(data);
        }
        self.buffer.reverse();
        return result;
    }

    // pub fn write_num(&mut self, num: u32) {
    //     self.buffer.clear(); // Empty it out
    //     let error_char = font8x8::BOX_UNICODE[115].byte_array(); // ╳
    //     for char in chars.chars() {
    //         let mut data = font8x8::LATIN_FONTS
    //             .get(char)
    //             .or_else(|| font8x8::BLOCK_FONTS.get(char))
    //             .or_else(|| font8x8::MISC_FONTS.get(char))
    //             .or_else(|| font8x8::GREEK_FONTS.get(char))
    //             .or_else(|| font8x8::BOX_FONTS.get(char))
    //             .or_else(|| font8x8::SGA_FONTS.get(char))
    //             .or_else(|| font8x8::HIRAGANA_FONTS.get(char))
    //             .or_else(|| font8x8::BASIC_FONTS.get(char))
    //             .unwrap_or_else(|| error_char);
    //         if config::DISPLAY_VERTICAL_FLIP == 1 {
    //             data.reverse();
    //         }
    //         self.buffer.push(data).unwrap();
    //     }
    //     self.buffer.reverse();
    // }


    /// Meant to be called once every millisecond, takes care of rotating the active
    /// `Message` and removing any that have timed out.
    pub fn tick(&mut self) -> Result<(), [u8; 8]> {
        let mut to_remove = None;
        let mut activate_message = None;
        if self.messages.len() == 0 { return Ok(()); } // Nothing to do
        // Count down each message's time, ignoring any that have `time == 0`
        for (i, message) in self.messages.iter_mut().enumerate() {
            if message.time >= 1 {
                if message.active {
                    message.time = message.time.saturating_sub(1);
                    if message.time == 1 {
                        to_remove = Some(i);
                    }
                    break;
                } else {
                    // Mark it as active so it will be swapped with the current message
                    message.active = true;
                    activate_message = Some(message.clone());
                }
            }
        }
        // Yeah this only does one at a time but we'll get any other "done" messages on the next tick
        let mut result = Ok(());
        if let Some(i) = to_remove {
            let _ = self.messages.swap_remove(i);
        } else if let Some(message) = activate_message {
            result = self.write_string(&message.text);
            // Mark the default message as inactive too:
            self.messages[0].active = false;
        }
        if self.messages.len() == 1 { // Only the default message remaining
            if self.messages[0].active == false {
                self.messages[0].active = true; // Make it the active message again
                let mut string: String<{ BUFFER_LENGTH }> = String::new();
                let _ = string.push_str(&self.messages[0].text.clone());
                result = self.write_string(&string)
            }
        }
        return result;
    }

    /// Shifts all dots in the matrix to the left by one
    pub fn shift_left(&mut self) {
        let buffer_length = self.buffer.len();
        if config::DISPLAY_VERTICAL_FLIP == 1 {
            // Shift to the right (cuz we're flipped)
            for disp in (0..buffer_length).into_iter().rev() {
                for row in 0..8 {
                    if self.buffer[disp][row] & 0b00000001 != 0 {
                        if disp == buffer_length - 1 {
                            // Tack it on to the far left
                            self.buffer[0][row] |= 0b10000000;
                        } else {
                            // Tack it on to the next matrix
                            self.buffer[disp + 1][row] |= 0b10000000;
                        }
                    }
                    self.buffer[disp][row] = self.buffer[disp][row] >> 1;
                }
            }
        } else {
            // Shift whole row to the left
            for disp in 0..buffer_length {
                for row in 0..8 {
                    if self.buffer[disp][row] & 0b10000000 != 0 {
                        if disp == 0 {
                            // Tack it on to the far right
                            self.buffer[buffer_length - 1][row] |= 1;
                        } else {
                            // Tack it on to the previous matrix
                            self.buffer[disp - 1][row] |= 1;
                        }
                    }
                    self.buffer[disp][row] = self.buffer[disp][row] << 1;
                }
            }
        }
    }

    /// Shifts all dots in the matrix to the right by one
    pub fn shift_right(&mut self) {
        let buffer_length = self.buffer.len();
        if config::DISPLAY_VERTICAL_FLIP == 1 {
            // Shift to the left (cuz we're flipped)
            for disp in 0..buffer_length {
                for row in 0..8 {
                    if self.buffer[disp][row] & 0b10000000 != 0 {
                        if disp == 0 {
                            // Tack it on to the far right
                            self.buffer[buffer_length - 1][row] |= 1;
                        } else {
                            // Tack it on to the previous matrix
                            self.buffer[disp - 1][row] |= 1;
                        }
                    }
                    self.buffer[disp][row] = self.buffer[disp][row] << 1;
                }
            }
        } else {
            // Shift to the right
            for disp in (0..buffer_length).into_iter().rev() {
                for row in 0..8 {
                    if self.buffer[disp][row] & 0b00000001 != 0 {
                        if disp == buffer_length - 1 {
                            // Tack it on to the far left
                            self.buffer[0][row] |= 0b10000000;
                        } else {
                            // Tack it on to the next matrix
                            self.buffer[disp + 1][row] |= 0b10000000;
                        }
                    }
                    self.buffer[disp][row] = self.buffer[disp][row] >> 1;
                }
            }
        }
    }

    pub fn sync(&mut self) {
        for disp in 0..self.num_matrices {
            self.display.write_raw(disp, &self.buffer[disp]).unwrap();
        }
    }
}
