//! The equivalent of Keyberon's matrix.rs but for Hall Effect sensors
//! connected to analog multilpexers

use core::ops::{Index, IndexMut};
// use embedded_hal::adc::Channel;
use crate::config;

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
    pub low: u16, // Keep track of the lowest value
    pub high: u16, // Keep track of the highest value
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
        if val > self.high { self.high = val; }
        if val < self.low { self.low = val; }
    }
}

/// Struct for storing the state of each channel and pretty-printing it via rprintln
#[derive(Debug, Default, Clone)]
pub struct ChannelStates {
    pub states: [ChannelState; config::KEYBOARD_MAX_CHANNELS],
    curr: usize, // Iterator tracking
    next: usize, // Ditto
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
