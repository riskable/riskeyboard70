//! The equivalent of Keyberon's matrix.rs but for Hall Effect sensors
//! connected to analog multilpexers

use core::fmt::Write;
use core::ops::{Index, IndexMut};
// use embedded_hal::adc::Channel;
use crate::config;
use crate::layers;
use keyberon::layout::{Event, Layout};
use rtt_target::UpChannel;

// Disabled smoothing for now since it doesn't seem to help much:
// extern crate arraydeque;

// use arraydeque::ArrayDeque;
// use arraydeque::behavior::Wrapping;

// use embedded_hal::adc::Channel;
// use generic_array::{ArrayLength, GenericArray};
// use heapless::Vec;
// use crate::layout::Event;

// const SMOOTHING: usize = 5; // Number of values to keep when calculating rolling average

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelState {
    pub pressed: bool,
    pub value: u16,
    pub default: u16,
    // pub smoothed: ArrayDeque<[u16; SMOOTHING], Wrapping>,
    // These are so we can track/debug voltage wobble:
    pub low: u16,     // Keep track of the lowest value
    pub high: u16,    // Keep track of the highest value
    pub rising: bool, // Whether or not this channel is rising (only used by rotary encoders)
}

impl Default for ChannelState {
    fn default() -> ChannelState {
        ChannelState {
            pressed: false,
            value: 0,
            default: 0,
            // smoothed: ArrayDeque::new(),
            low: 3200,
            high: 0,
            rising: false,
        }
    }
}

impl ChannelState {
    /// Sets self.pressed to true
    pub fn press(&mut self) {
        self.pressed = true;
    }

    /// Sets self.pressed to false
    pub fn release(&mut self) {
        self.pressed = false;
    }

    /// Updates self.default with the given value
    pub fn update_default(&mut self, val: u16) {
        self.default = val;
    }

    /// Records the given value, making sure to record any lows or highs.
    /// If *default* is true then the value will be recorded as the default value.
    pub fn record_value(&mut self, val: u16) {
        self.value = val;
        if val > self.high {
            self.high = val;
        }
        if val < self.low {
            self.low = val;
        }
    }
}

/// Struct for storing the state of each channel and pretty-printing it via rprintln
#[derive(Debug, Default, Clone)]
pub struct ChannelStates {
    pub states: [ChannelState; config::KEYBOARD_MAX_CHANNELS],
    curr: usize,        // Iterator tracking
    next: usize,        // Ditto
    pub pressed: usize, // Records how many keys are currently pressed
}

impl ChannelStates {
    pub fn update_default_by_index(&mut self, chan: usize, val: u16) {
        self.states[chan].default = val;
    }
    pub fn update_rising_by_index(&mut self, chan: usize, val: bool) {
        self.states[chan].rising = val;
    }
    pub fn press(&mut self, chan: usize) {
        self.states[chan].press();
        self.pressed_add();
    }
    pub fn release(&mut self, chan: usize) {
        self.states[chan].release();
        self.pressed_sub();
    }
    pub fn pressed_add(&mut self) {
        self.pressed += 1;
    }
    pub fn pressed_sub(&mut self) {
        self.pressed -= 1;
    }
}

impl Iterator for ChannelStates {
    type Item = ChannelState;

    fn next(&mut self) -> Option<ChannelState> {
        if self.curr < config::KEYBOARD_MAX_CHANNELS {
            self.curr = self.curr.saturating_add(1);
        } else {
            return None;
        }
        Some(self.states[self.curr])
    }
}

impl Index<usize> for ChannelStates {
    type Output = ChannelState;

    fn index(&self, i: usize) -> &ChannelState {
        &self.states[i]
    }
}

impl IndexMut<usize> for ChannelStates {
    fn index_mut<'a>(&'a mut self, i: usize) -> &'a mut ChannelState {
        &mut self.states[i]
    }
}

// impl our super user-friendly terminal view into all channels
impl core::fmt::Display for ChannelStates {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        // let _ = f.write_str("\x1B[2J\x1B[0H"); // Clear the screen and move cursor to start
        let _ = f.write_str("Multiplexer Channel Values:\n");
        let _ = f.write_str("\nch0    ch1    ch2    ch3    ch4    ch5    ch6    ch7\n");
        for i in 0..8 {
            let _ = f
                .write_fmt(format_args!("{}   ", self.states[i].value))
                .unwrap();
        }
        let _ = f.write_str("\n");
        let _ = f.write_str("ch8    ch9    ch10   ch11   ch12   ch13   ch14   ch15\n");
        for i in 8..10 {
            let _ = f
                .write_fmt(format_args!("{}   ", self.states[i].value))
                .unwrap();
        }
        for i in 10..16 {
            let _ = f
                .write_fmt(format_args!("{}    ", self.states[i].value))
                .unwrap();
        }
        let _ = f.write_str("\n");
        Ok(())
    }
}

///! Updates the status of all the Keyberon stuff in *layout* and returns true if a NEW keypress or rotary encoder movement was detected
pub fn check_channel(
    multilpexer: usize,
    chan: usize,
    millivolts: u16,
    ch_states: &mut [ChannelStates],
    layout: &mut Layout<layers::CustomActions>,
    rotary_clockwise: &mut bool,
    actuation_threshold: u16,
    encoder_press_threshold: u16,
    release_threshold: u16,
    _debug_ch0: &mut UpChannel,
) -> bool {
    let ch_state = ch_states[multilpexer][chan];
    if ch_state.value > config::KEYBOARD_IGNORE_BELOW {
        let voltage_difference = if millivolts < ch_state.default {
            if config::KEYBOARD_NORTH_DOWN > 0 {
                ch_state.default - millivolts // North side down switches result in a mV drop
            } else {
                0
            }
        } else {
            if config::KEYBOARD_NORTH_DOWN > 0 {
                0
            } else {
                millivolts - ch_state.default // South side down switches result in a mV increase
            }
        };
        // let voltage_difference = if millivolts < ch_state.default {
        //     ch_state.default - millivolts // North side down switches result in a mV drop
        // } else {
        //     // millivolts - ch_state.default // South side down switches result in a mV increase
        //     0
        // };
        // let _ = writeln!(_debug_ch0, "voltage_difference: {:?}", voltage_difference);
        // Handle the rotary encoders since they're special
        /* NOTE: Here's how the rotary encoder logic works:
            * ENCODER1's action in Layers is used for clockwise movements (so we can use Keyberon's event system)
            * ENCODER2's action in Layers is used for counter-clockwise movements
            * The unused sensor after ENCODER2 is used for ENCODER_PRESS (RotaryPress) which presently isn't used

            Change direction (enc1 and enc2 rising):
                false false -> true true
                true true -> false false
                false true -> true false
                true false -> false true

            Always CW:
                false false -> true false
                true false -> true true
                true true -> false true
                false true -> false false

            Always CCW:
                false false -> false true
                false true -> true true
                true true -> true false
                true false -> false false
        */
        if multilpexer == config::ENCODER_MUX {
            if chan == config::ENCODER_CHANNEL2 {
                let cw_event_chan = config::ENCODER_CHANNEL1 as u8;
                let ccw_event_chan = config::ENCODER_CHANNEL2 as u8;
                let encoder_press_chan = config::ENCODER_PRESS_CHANNEL as u8;
                // The rotary encoder values could go up or down depending on which side
                // of the magnets are over the sensors
                let encoder2_voltage_difference = if millivolts < ch_state.default {
                    ch_state.default - millivolts
                } else {
                    millivolts - ch_state.default
                };
                let enc1 = ch_states[multilpexer][cw_event_chan as usize];
                let enc2 = ch_states[multilpexer][ccw_event_chan as usize];
                let enc1_rising = if enc1.value > enc1.default {
                    true
                } else {
                    false
                };
                let enc2_rising = if enc2.value > enc2.default {
                    true
                } else {
                    false
                };
                // Get encoder1's voltage difference too so we can detect a press event properly
                let encoder1_voltage_difference = if enc1.value < enc1.default {
                    enc1.default - enc1.value
                } else {
                    enc1.value - enc1.default
                };
                // let _ = writeln!(_debug_ch0, "enc1_mv: {:?} enc2_mv: {:?}", enc1.value, enc2.value);
                // let _ = writeln!(_debug_ch0, "enc1.low: {:?} enc2.low: {:?}", enc1.low, enc2.low);
                // Encoder Press() events are disabled for now because it's probably impossible to detect reliably
                // with the current PCB design.
                // // Encoder Press() needs to be handled before all other encoder stuff
                // let combined_encoder_voltage_difference =
                //     encoder1_voltage_difference + encoder2_voltage_difference;
                // if combined_encoder_voltage_difference > encoder_press_threshold {
                //     let _ = writeln!(
                //         _debug_ch0,
                //         "OVER PRESS THRESHOLD: Combined_encoder_voltage_difference: {:?}",
                //         combined_encoder_voltage_difference
                //     );
                //     if enc1.pressed {
                //         ch_states[multilpexer][cw_event_chan as usize].release();
                //         ch_states[multilpexer][cw_event_chan as usize].default = enc1.value;
                //         let _ =
                //             layout.event(Event::Release(multilpexer as u8, cw_event_chan as u8));
                //     }
                //     if enc2.pressed {
                //         ch_states[multilpexer][ccw_event_chan as usize].release();
                //         ch_states[multilpexer][ccw_event_chan as usize].default = enc2.value;
                //         let _ =
                //             layout.event(Event::Release(multilpexer as u8, ccw_event_chan as u8));
                //     }
                //     if ch_states[multilpexer][encoder_press_chan as usize].pressed {
                //         return true; // It's still pressed
                //     } else {
                //         let _ = writeln!(
                //             _debug_ch0,
                //             "encoder press 1: {} 2: {}",
                //             encoder1_voltage_difference, encoder2_voltage_difference
                //         );
                //         ch_states[multilpexer][encoder_press_chan as usize].press();
                //         let _ =
                //             layout.event(Event::Press(multilpexer as u8, encoder_press_chan as u8));
                //         // Store the combined default on the virtual channel
                //         ch_states[multilpexer].update_default_by_index(
                //             encoder_press_chan as usize,
                //             combined_encoder_voltage_difference,
                //         );
                //         // Update the defaults so we get correct comparisons after
                //         ch_states[multilpexer]
                //             .update_default_by_index(cw_event_chan as usize, enc1.value);
                //         ch_states[multilpexer]
                //             .update_default_by_index(ccw_event_chan as usize, enc2.value);
                //         return true;
                //     }
                // } else if ch_states[multilpexer][encoder_press_chan as usize].pressed {
                //     let _ = writeln!(
                //         _debug_ch0,
                //         "encoder release 1: {} 1def: {} 2: {} 2def: {}, combined diff: {}",
                //         enc1.value, enc1.default, enc2.value, enc2.default, combined_encoder_voltage_difference
                //     );
                //     ch_states[multilpexer][encoder_press_chan as usize].release();
                //     let _ =
                //         layout.event(Event::Release(multilpexer as u8, encoder_press_chan as u8));
                //     // Update the defaults so we get correct comparisons after
                //     ch_states[multilpexer]
                //         .update_default_by_index(cw_event_chan as usize, enc1.value);
                //     ch_states[multilpexer]
                //         .update_default_by_index(ccw_event_chan as usize, enc2.value);
                //     return false;
                // }
                if encoder1_voltage_difference > config::ENCODER_RESOLUTION
                    || encoder2_voltage_difference > config::ENCODER_RESOLUTION
                {
                    // let _ = writeln!(_debug_ch0, "encoder_voltage_difference: {:?}", encoder_voltage_difference);
                    // let _ = writeln!(_debug_ch0, "enc1_mv: {:?} enc2_mv: {:?}", enc1.value, enc2.value);
                    // let _ = writeln!(_debug_ch0, "enc1_default: {:?} enc2_default: {:?}", enc1.default, enc2.default);
                    // let _ = writeln!(_debug_ch0, "enc1_rising: {:?} enc2_rising: {:?}", enc1_rising, enc2_rising);
                    if enc1_rising && enc2_rising {
                        // true true
                        if enc1.rising && !enc2.rising {
                            // Previous true false means clockwise
                            *rotary_clockwise = true;
                            ch_states[multilpexer][cw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                        } else if !enc1.rising && enc2.rising {
                            // Previous false true means counterclockwise
                            *rotary_clockwise = false;
                            ch_states[multilpexer][ccw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                        } else if !enc1.rising && !enc2.rising {
                            // Change of direction
                            if *rotary_clockwise {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            } else {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            }
                        } else if enc1.rising && enc2.rising {
                            // Continue our present direction
                            if *rotary_clockwise {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            } else {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            }
                        }
                    } else if !enc1_rising && !enc2_rising {
                        // false false
                        if !enc1.rising && enc2.rising {
                            // Previous false true means clockwise
                            *rotary_clockwise = true;
                            ch_states[multilpexer][cw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                        } else if enc1.rising && !enc2.rising {
                            // Previous true false means counterclockwise
                            *rotary_clockwise = false;
                            ch_states[multilpexer][ccw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                        } else if enc1.rising && enc2.rising {
                            // Change of direction
                            if *rotary_clockwise {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            } else {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            }
                        } else if !enc1.rising && !enc2.rising {
                            // Continue our present direction
                            if *rotary_clockwise {
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            } else {
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            }
                        }
                    } else if !enc1_rising && enc2_rising {
                        // false true
                        if enc1.rising && enc2.rising {
                            // Previous true true means clockwise
                            *rotary_clockwise = true;
                            ch_states[multilpexer][cw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                        } else if !enc1.rising && !enc2.rising {
                            // Previous false false means counterclockwise
                            *rotary_clockwise = false;
                            ch_states[multilpexer][ccw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                        } else if enc1.rising && !enc2.rising {
                            // Change of direction
                            if *rotary_clockwise {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            } else {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            }
                        } else if !enc1.rising && enc2.rising {
                            // Continue our present direction
                            if *rotary_clockwise {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            } else {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            }
                        }
                    } else if enc1_rising && !enc2_rising {
                        // true false
                        if !enc1.rising && !enc2.rising {
                            // Previous false false means clockwise
                            *rotary_clockwise = true;
                            ch_states[multilpexer][cw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                        } else if enc1.rising && enc2.rising {
                            // Previous true true means counterclockwise
                            *rotary_clockwise = false;
                            ch_states[multilpexer][ccw_event_chan as usize].press();
                            let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                        } else if !enc1.rising && enc2.rising {
                            // Change of direction
                            if *rotary_clockwise {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            } else {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            }
                        } else if enc1.rising && !enc2.rising {
                            // Continue our present direction
                            if *rotary_clockwise {
                                *rotary_clockwise = true;
                                ch_states[multilpexer][cw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                            } else {
                                *rotary_clockwise = false;
                                ch_states[multilpexer][ccw_event_chan as usize].press();
                                let _ =
                                    layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                            }
                        }
                    }
                    // Reset to current value
                    // let _ = writeln!(
                    //     _debug_ch0,
                    //     "Updating defaults to: enc1: {:?} enc2: {:?}",
                    //     enc1.value, enc2.value
                    // );
                    ch_states[multilpexer]
                        .update_default_by_index(cw_event_chan as usize, enc1.value);
                    ch_states[multilpexer]
                        .update_default_by_index(ccw_event_chan as usize, enc2.value);
                    ch_states[multilpexer]
                        .update_rising_by_index(cw_event_chan as usize, enc1_rising);
                    ch_states[multilpexer]
                        .update_rising_by_index(ccw_event_chan as usize, enc2_rising);
                    return true;
                } else {
                    // Reset pressed state immediately ("pressed" doesn't make sense for rotational movements)
                    if enc1.pressed {
                        ch_states[multilpexer][cw_event_chan as usize].release();
                        ch_states[multilpexer][cw_event_chan as usize].default = enc1.value;
                        // let _ = writeln!(_debug_ch0, "CW release");
                        let _ =
                            layout.event(Event::Release(multilpexer as u8, cw_event_chan as u8));
                    }
                    if enc2.pressed {
                        ch_states[multilpexer][ccw_event_chan as usize].release();
                        ch_states[multilpexer][ccw_event_chan as usize].default = enc2.value;
                        // let _ = writeln!(_debug_ch0, "CCW release");
                        let _ =
                            layout.event(Event::Release(multilpexer as u8, ccw_event_chan as u8));
                    }
                    if ch_states[multilpexer][encoder_press_chan as usize].pressed {
                        let _ = writeln!(_debug_ch0, "Encoder press release");
                        ch_states[multilpexer][encoder_press_chan as usize].release();
                        ch_states[multilpexer][cw_event_chan as usize].default = enc1.value;
                        ch_states[multilpexer][ccw_event_chan as usize].default = enc2.value;
                        let _ = layout
                            .event(Event::Release(multilpexer as u8, encoder_press_chan as u8));
                    }
                    return false;
                }
            }
        }
        // Handle normal keypresses
        if voltage_difference > actuation_threshold {
            if !ch_state.pressed {
                // let _ = writeln!(_debug_ch0,
                //     "Triggering Press event for ADC pin: PA{}, channel {} ({})",
                //     multilpexer,
                //     chan,
                //     voltage_difference);
                // Encoder press doesn't work very reliably (needs work--probably a change to the PCB):
                if multilpexer == config::ENCODER_MUX {
                    if chan != config::ENCODER_CHANNEL1 && chan != config::ENCODER_CHANNEL2 {
                        ch_states[multilpexer].press(chan);
                        let _ = layout.event(Event::Press(multilpexer as u8, chan as u8));
                    }
                } else {
                    ch_states[multilpexer].press(chan);
                    let _ = layout.event(Event::Press(multilpexer as u8, chan as u8));
                }
                return true;
            }
        } else if voltage_difference < release_threshold {
            if ch_state.pressed {
                // let _ = writeln!(_debug_ch0,
                //     "Triggering Release event for ADC pin: PA{}, channel {} ({})",
                //     multilpexer,
                //     chan,
                //     voltage_difference);
                if multilpexer == config::ENCODER_MUX {
                    if chan != config::ENCODER_CHANNEL1 && chan != config::ENCODER_CHANNEL2 {
                        ch_states[multilpexer].release(chan);
                        let _ = layout.event(Event::Release(multilpexer as u8, chan as u8));
                    }
                } else {
                    ch_states[multilpexer].release(chan);
                    let _ = layout.event(Event::Release(multilpexer as u8, chan as u8));
                }
                return false;
            }
        }
    }
    false
}
