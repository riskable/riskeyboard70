//! Functions related to the analog hall effect rotary encoder

use keyberon::layout::{Event, Layout};
use rtt_target::UpChannel;

use crate::config;
use crate::layers;
use crate::multiplexers;

// The channels on the analog multiplexer where the encoder sensors reside
pub const ENCODER_CHANNEL1: usize = 8;
pub const ENCODER_CHANNEL2: usize = 9;
pub const ENCODER_PRESS: usize = 10; // An unsed channel that's used for rotary encoder button press events

/// Handles the rotary encoder by checking the state of its two hall effect sensors
/// then sending appropriate Press/Release events depending on the detected
/// rotational direction (clockwise or counterclockwise)
pub fn handle_encoder(
    multilpexer: usize,
    chan: usize,
    millivolts: u16,
    ch_states: &mut [multiplexers::ChannelStates],
    layout: &mut Layout<layers::CustomActions>,
    encoder_voltage_difference: u16,
    rotary_clockwise: &mut bool,
    debug_ch0: &mut UpChannel,
) -> bool {
    let enc1 = ch_states[multilpexer].by_index(ENCODER_CHANNEL1).clone();
    let enc2 = ch_states[multilpexer].by_index(ENCODER_CHANNEL2).clone();
    let enc1_rising = if enc1.value > enc1.default { true } else { false };
    let enc2_rising = if enc2.value > enc2.default { true } else { false };
    // let _ = writeln!(debug_ch0, "enc1_mv: {:?} enc2_mv: {:?}", enc1.value, enc2.value);
    // let _ = writeln!(debug_ch0, "enc1.low: {:?} enc2.low: {:?}", enc1.low, enc2.low);
    if enc1.value > enc1.low && enc1.value - enc1.low < config::ACTUATION_THRESHOLD {
        ch_states[multilpexer].press(ENCODER_PRESS as usize);
        let _ = layout.event(Event::Release(multilpexer as u8, ENCODER_PRESS as u8));
        return true;
    }
    if encoder_voltage_difference > ENCODER_RESOLUTION {
        // let _ = writeln!(debug_ch0, "enc1_mv: {:?} enc2_mv: {:?}", enc1.value, enc2.value);
        // let _ = writeln!(debug_ch0, "enc1_default: {:?} enc2_default: {:?}", enc1_prev_default, enc2.default);
        // let _ = writeln!(debug_ch0, "enc1_rising: {:?} enc2_rising: {:?}", enc1_rising, enc2_rising);
        if enc1_rising && enc2_rising { // true true
            if enc1.rising && !enc2.rising { // Previous true false means clockwise
                *rotary_clockwise = true;
                ch_states[multilpexer].press(cw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
            } else if !enc1.rising && enc2.rising { // Previous false true means counterclockwise
                *rotary_clockwise = false;
                ch_states[multilpexer].press(ccw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
            } else if !enc1.rising && !enc2.rising { // Change of direction
                if *rotary_clockwise {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                } else {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                }
            } else if enc1.rising && enc2.rising { // Continue our present direction
                if *rotary_clockwise {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                } else {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                }
            }
        } else if !enc1_rising && !enc2_rising { // false false
            if !enc1.rising && enc2.rising { // Previous false true means clockwise
                *rotary_clockwise = true;
                ch_states[multilpexer].press(cw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
            } else if enc1.rising && !enc2.rising { // Previous true false means counterclockwise
                *rotary_clockwise = false;
                ch_states[multilpexer].press(ccw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
            } else if enc1.rising && enc2.rising { // Change of direction
                if *rotary_clockwise {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                } else {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                }
            } else if !enc1.rising && !enc2.rising { // Continue our present direction
                if *rotary_clockwise {
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                } else {
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                }
            }
        } else if !enc1_rising && enc2_rising { // false true
            if enc1.rising && enc2.rising { // Previous true true means clockwise
                *rotary_clockwise = true;
                ch_states[multilpexer].press(cw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
            } else if !enc1.rising && !enc2.rising { // Previous false false means counterclockwise
                *rotary_clockwise = false;
                ch_states[multilpexer].press(ccw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
            } else if enc1.rising && !enc2.rising { // Change of direction
                if *rotary_clockwise {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                } else {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                }
            } else if !enc1.rising && enc2.rising { // Continue our present direction
                if *rotary_clockwise {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                } else {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                }
            }
        } else if enc1_rising && !enc2_rising { // true false
            if !enc1.rising && !enc2.rising { // Previous false false means clockwise
                *rotary_clockwise = true;
                ch_states[multilpexer].press(cw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
            } else if enc1.rising && enc2.rising { // Previous true true means counterclockwise
                *rotary_clockwise = false;
                ch_states[multilpexer].press(ccw_event_chan as usize);
                let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
            } else if !enc1.rising && enc2.rising { // Change of direction
                if *rotary_clockwise {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                } else {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                }
            } else if enc1.rising && !enc2.rising { // Continue our present direction
                if *rotary_clockwise {
                    *rotary_clockwise = true;
                    ch_states[multilpexer].press(cw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, cw_event_chan));
                } else {
                    *rotary_clockwise = false;
                    ch_states[multilpexer].press(ccw_event_chan as usize);
                    let _ = layout.event(Event::Press(multilpexer as u8, ccw_event_chan));
                }
            }
        }
        // Reset to current value
        // let _ = writeln!(debug_ch0, "Restting default values enc1 to {} enc2 to {}", enc1.value, enc2.value);
        ch_states[multilpexer].update_default_by_index(cw_event_chan as usize, enc1.value);
        ch_states[multilpexer].update_default_by_index(ccw_event_chan as usize, enc2.value);
        ch_states[multilpexer].update_rising_by_index(cw_event_chan as usize, enc1_rising);
        ch_states[multilpexer].update_rising_by_index(ccw_event_chan as usize, enc2_rising);
        return true;
    } else {
        // Reset pressed state immediately ("pressed" doesn't make sense for rotational movements)
        if enc1.pressed {
            // ch_states[multilpexer].update_default_by_index(chan, millivolts); // Reset to current value
            ch_states[multilpexer].release(cw_event_chan as usize);
            ch_states[multilpexer].update_default_by_index(cw_event_chan as usize, enc1.value);
            let _ = layout.event(Event::Release(multilpexer as u8, cw_event_chan as u8));
        }
        if enc2.pressed {
            ch_states[multilpexer].release(ccw_event_chan as usize);
            ch_states[multilpexer].update_default_by_index(ccw_event_chan as usize, enc2.value);
            let _ = layout.event(Event::Release(multilpexer as u8, ccw_event_chan as u8));
        }
        if ch_states[multilpexer].by_index(ENCODER_PRESS as usize).pressed {
            ch_states[multilpexer].release(ENCODER_PRESS as usize);
            let _ = layout.event(Event::Release(multilpexer as u8, ENCODER_PRESS as u8));
        }
        return false;
    }
}