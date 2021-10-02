//! Holds our default (initial) keyboard layout/actions
#![allow(dead_code)]
use keyberon::action::{k, l, m, Action, Action::*, SequenceEvent::*, HoldTapConfig};
use keyberon::key_code::KeyCode::*;
use infrared::remotecontrol::Button as IRButton;

use crate::ir_remote::IRRemote;

/// All our custom Keyberon actions
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum CustomActions {
    /// LEDs Brighter
    LEDUp,
    /// LEDs Darker
    LEDDown,
    /// Toggle the LEDs on/off
    LEDToggle,
    /// Display brighter
    DisplayUp,
    /// Display darker
    DisplayDown,
    /// Toggle the display on/off
    DisplayToggle,
    /// Toggle all lighting on/off
    LightingToggle,
    /// Mouse button 1 (left)
    Mouse1,
    /// Mouse button 2 (middle)
    Mouse2,
    /// Mouse button 3 (right)
    Mouse3,
    /// Mousewheel forwards
    Mouse4,
    /// Mousewheel back
    Mouse5,
    /// Infrared remote control codes/button mappings
    IRRemote,
    /// Reset the MCU
    Reset,
}

// LED brightness control
const LEDS_BRIGHTER: Action<CustomActions> = Custom(CustomActions::LEDUp);
const LEDS_DARKER: Action<CustomActions> = Custom(CustomActions::LEDDown);
const LEDS_TOGGLE: Action<CustomActions> = Custom(CustomActions::LEDToggle);
// Display brightness control
const DISP_BRIGHTER: Action<CustomActions> = Custom(CustomActions::DisplayUp);
const DISP_DARKER: Action<CustomActions> = Custom(CustomActions::DisplayDown);
const DISP_TOGGLE: Action<CustomActions> = Custom(CustomActions::DisplayToggle);
// Toggle both LEDs and Display
const LIGHTING_TOGGLE: Action<CustomActions> = Custom(CustomActions::LightingToggle);
// Mouse wheel emulation
const SCROLLUP: Action<CustomActions> = Custom(CustomActions::Mouse4);
const SCROLLDOWN: Action<CustomActions> = Custom(CustomActions::Mouse5);
// So we can reboot the keeb into DFU bootloader mode
const RESET: Action<CustomActions> = Custom(CustomActions::Reset);
// IR Button Mapping
// const IRPOWER: Action<CustomActions> = Custom(CustomActions::IRRemote(IRButton::Power));

// const FIVEO: Action = Sequence {
//     events: &[
//         Press(Kb0),
//         Release(Kb0),
//         Repeat {
//             times: 4,
//             events: &[Press(Kb0),Release(Kb0)],
//         },
//     ],
// };

const HUNDRED: Action<CustomActions> = Sequence {
    events: &[ // Types 💯
        Press(LCtrl),
        Press(LShift),
        Press(U),
        Tap(Kb1),
        Tap(F),
        Tap(Kb4),
        Tap(A),
        Tap(F),
        Complete,
        // These should be unnecessary thanks to the `Complete` above
        // Release(LCtrl),
        // Release(LShift),
        // Release(U),
    ],
};
const THUMBSUP: Action<CustomActions> = Sequence {
    events: &[ // Types 👍
        Press(LCtrl),
        Press(LShift),
        Press(U),
        Tap(Kb1),
        Tap(F),
        Tap(Kb4),
        Tap(Kb4),
        Tap(D),
        Complete,
    ],
};
const TOPARROW: Action<CustomActions> = Sequence {
    events: &[ // Types 🔝
        Press(LCtrl),
        Press(LShift),
        Press(U),
        Tap(Kb1),
        Tap(F),
        Tap(Kb5),
        Tap(Kb1),
        Tap(D),
        Complete,
    ],
};
const OMEGA: Action<CustomActions> = Sequence {
    events: &[ // Types Ω
        Press(LCtrl),
        Press(LShift),
        Press(U),
        Tap(Kb2),
        Tap(Kb1),
        Tap(Kb2),
        Tap(Kb6),
        Complete,
    ],
};
const MACROTEST2: Action<CustomActions> = Sequence {
    events: &[
        Press(LShift),
        Press(Kb1),
        Release(Kb1),
        Tap(Kb2),
        Tap(Kb3),
        Tap(Kb4),
        Tap(Kb5),
        Tap(Kb6),
        Tap(Kb7),
        Tap(Kb8),
        Delay {duration: 1000},
        Tap(Kb2),
        Delay {duration: 1000},
        Tap(Kb3),
        Delay {duration: 1000},
        Tap(Kb4),
        Delay {duration: 1000},
        Tap(Kb5),
        Delay {duration: 1000},
        Tap(Kb6),
        Complete,
    ],
};
// Makes sure we don't send multiple MediaCalc events via keyrepeat
const ONESHOT_CALC: Action<CustomActions> = Sequence {
    events: &[
        Tap(MediaCalc),
        Delay {duration: 1000},
    ],
};
const GRAVE_AND_CALC: Action<CustomActions> = HoldTap {
    timeout: 300,
    tap_hold_interval: 0,
    config: HoldTapConfig::Default,
    hold: &ONESHOT_CALC,
    tap: &k(Grave),
};
const PLUS_ONE_THUMBSUP: Action<CustomActions> = HoldTap {
    timeout: 300,
    tap_hold_interval: 0,
    config: HoldTapConfig::Default,
    hold: &THUMBSUP,
    tap: &k(KpPlus),
};
const L1_FUN: Action<CustomActions> = HoldTap {
    timeout: 300,
    tap_hold_interval: 0,
    config: HoldTapConfig::Default,
    hold: &l(1),
    tap: &k(Space),
};
const L2_FUN2: Action<CustomActions> = HoldTap {
    timeout: 300,
    tap_hold_interval: 0,
    config: HoldTapConfig::Default,
    hold: &l(2),
    tap: &k(KpEnter),
};
// Prevents a long-press of the PrintScreen key from opening five zillion Spectacle windows
const SCREENSHOT_FIX: Action<CustomActions> = HoldTap {
    timeout: 300,
    tap_hold_interval: 0,
    config: HoldTapConfig::Default,
    hold: &Trans, // Do nothing
    tap: &k(PScreen),
};
// const CSPACE: Action<CustomActions> = m(&[LCtrl, Space]);
// macro_rules! s {
//     ($k:ident) => {
//         m(&[LShift, $k])
//     };
// }
// macro_rules! a {
//     ($k:ident) => {
//         m(&[RAlt, $k])
//     };
// }

// NOTE: What most folks consider the "Menu" key is actually the "Application" key in Keyberon./
// TODO: Figure out a way to read this in from a config file
// TODO: Also figure out how to make this configurable via the USB serial port
// NOTE: For the prototype we're using a 4x6-ish numpad but the keys don't have a nice 1-to-1 mapping
//       of multiplexer-pin-to-key.  This was because routing on the PCB required compromises.
//       Here's the schema mapping on the PCB:
/* The mapping is in the order of the keys with the designatin: <multiplexer>:<channel>

    0:7, 0:9, 1:6, 1:1,
    0:6, 0:10, 1:5, 1:15,
    0:4, 0:11, 1:4,
                    1:14,
    0:14, 0:12, 1:3,
                    1:12,
    0:15, 0:13, 1:2,
    0:0,    1:0

*/
#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<CustomActions> = &[
    /*
    Since Keyberon was made for key switch matrices and not multi-channel analog multiplexers
    our mapping below is vastly more arbitrary and based on the tracks of the PCB rather than
    anything easy like rows/columns.  To properly translate this you need to figure out which
    key maps to what multiplexer-channel combo.
    Example: For our numpad multiplexer 0, channel 0 connects to the "0" key.  So the very
    first k() translation goes to Kp0.

    */
    // For the sake of scanning, six 16-channel multiplexers are laid out where one
    // multiplexer is the equivalent to one row in a key matrix...
    &[ // Layer 0 (default layer)
        // AM0 (PC version)
        // &[k(Z),k(LAlt),k(LGui),k(LShift),k(LCtrl),k(CapsLock),k(Tab),GRAVE_AND_CALC,
        //     k(Escape),k(Kb1),k(Kb2),k(Q),k(A),k(W),k(S),Trans],
        // AM0 (Mac version Alt aka Option and LGui aka Command are swapped)
        &[k(Z),k(LGui),k(LAlt),k(LShift),k(LCtrl),k(CapsLock),k(Tab),GRAVE_AND_CALC,
            k(Escape),k(Kb1),k(Kb2),k(Q),k(A),k(W),k(S),Trans],
        // AM1
        &[k(R),k(Kb3),k(E),k(F),k(C),l(1),k(X),k(D),
            k(Kb4),k(Kb5),k(T),k(V),k(BSpace),k(Kb6),k(G),k(B)],
        // AM2
        &[l(2),k(M),k(J),k(Space),k(N),k(H),k(Y),k(Kb7),
            k(U),k(Kb8),k(Kb9),k(I),k(Comma),k(K),Trans,Trans],
        // AM3
        &[k(Slash),k(SColon),k(Kb0),k(P),k(RAlt),k(Dot),k(L),k(O),
            k(Minus),k(Equal),Trans,k(LBracket),k(RBracket),k(RCtrl),k(Quote),k(Application)],
        // AM4
        &[k(Right),k(Up),k(Down),k(Left),k(RShift),k(Enter),k(BSpace),k(Bslash),
            SCROLLDOWN, // Encoder clockwise
            SCROLLUP, // Encoder counterclockwise
            LIGHTING_TOGGLE, // Encoder press
            Trans,Trans, // Unused pins
            k(Delete),SCREENSHOT_FIX,k(Insert)], // Macro1, Macro2, and Macro3 (respectively)
        // This maps to your infrared remote BUTTONS array (see ir_remote.rs)
        &[
            LIGHTING_TOGGLE, // (69, Action::Power),
            k(Menu), // (71, Action::Menu),
            k(MediaFind), // (68, Action::Info) aka "TEST" button
            k(VolUp), // (64, Action::VolumeUp),
            k(VolDown), // (25, Action::VolumeDown),
            k(Escape), // (13, Action::ChannelList) aka "C" button
            k(MediaNextSong), // (9, Action::Next),
            k(MediaPreviousSong), // (7, Action::Prev),
            k(MediaPlayPause), // (21, Action::Play),
            k(MediaBack), // (67, Action::Rewind),
            k(Kp0), // (22, Action::Zero),
            k(Kp1), // (12, Action::One),
            k(Kp2), // (24, Action::Two),
            k(Kp3), // (94, Action::Three),
            k(Kp4), // (8, Action::Four),
            k(Kp5), // (28, Action::Five),
            k(Kp6), // (90, Action::Six),
            k(Kp7), // (66, Action::Seven),
            k(Kp8), // (82, Action::Eight),
            k(Kp9), // (74, Action::Nine),

            // Old remote
            // LIGHTING_TOGGLE, // (77, Button::Power),
            // Trans, // (84, Button::Source),
            // k(VolUp), // (30, Button::VolumeUp),
            // k(VolDown), // (10, Button::VolumeDown),
            // SCROLLUP, // (5, Button::Next),
            // SCROLLDOWN, // (2, Button::Prev),
            // k(Mute), // (22, Button::Mute),
            // Trans, // (76, Button::Rewind), // "Record"
            // k(F11), // "Full Screen"
            // m(&[LCtrl,LGui,LAlt,R]), // (12, Button::Time), // "Time Shift"
            // Trans, // (28, Button::Return), // "Recall"
            // Trans, // (18, Button::Zero),
            // Trans, // (9, Button::One),
            // Trans, // (29, Button::Two),
            // Trans, // (31, Button::Three),
            // Trans, // (13, Button::Four),
            // Trans, // (25, Button::Five),
            // Trans, // (27, Button::Six),
            // Trans, // (17, Button::Seven),
            // Trans, // (21, Button::Eight),
            // k(Kb9), // (23, Button::Nine),
        ],
    ], &[ // Layer 1 (Fun)
        // AM0
        &[k(Kb1),l(3),Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,k(F1),k(F2),Trans,Trans,Trans,Trans,Trans],
        // AM1
        &[Trans,k(F3),Trans,Trans,Trans,Trans,Trans,Trans,
            k(F4),k(F5),Trans,Trans,k(Delete),k(F6),Trans,Trans],
        // AM2
        &[l(2),Trans,Trans,k(Insert),Trans,HUNDRED,Trans,k(F7),
            Trans,k(F8),k(F9),Trans,Trans,Trans,Trans,Trans],
        // AM3
        &[Trans,Trans,Trans,Trans,l(5),Trans,Trans,OMEGA,
            k(F11),k(F12),Trans,Trans,Trans,Trans,Trans,Trans],
        // AM4
        &[k(End),k(PgUp),k(PgDown),k(Home),Trans,Trans,k(Delete),Trans,
            k(VolDown), // Encoder clockwise
            k(VolUp), // Encoder counterclockwise
            Trans, // Encoder press
            Trans,Trans, // Unused pins (grounded)
            RESET,k(ScrollLock),LIGHTING_TOGGLE], // Macro1, Macro2, and Macro3 (respectively)
        // Everything past this point is just for infrared stuff
        &[
            Trans, // (69, Action::Power)
            Trans, // (71, Action::Menu)
            Trans, // (68, Action::Info) aka "TEST" button
            Trans, // (64, Action::VolumeUp)
            Trans, // (25, Action::VolumeDown)
            Trans, // (13, Action::ChannelList) aka "C" button
            Trans, // (9, Action::Next)
            Trans, // (7, Action::Prev)
            Trans, // (21, Action::Play)
            Trans, // (67, Action::Rewind)
            Trans, // (22, Action::Zero)
            Trans, // (12, Action::One)
            Trans, // (24, Action::Two)
            Trans, // (94, Action::Three)
            Trans, // (8, Action::Four)
            Trans, // (28, Action::Five)
            Trans, // (90, Action::Six)
            Trans, // (66, Action::Seven)
            Trans, // (82, Action::Eight)
            Trans, // (74, Action::Nine)

            // Original remote I tested with
            // Trans, // (77, Button::Power),
            // Trans, // (84, Button::Source),
            // Trans, // (5, Button::Next),
            // Trans, // (2, Button::Prev),
            // Trans, // (22, Button::Mute),
            // Trans, // (76, Button::Rewind), // "Record"
            // Trans, // (12, Button::Time), // "Time Shift"
            // Trans, // (28, Button::Return), // "Recall"
            // Trans, // (18, Button::Zero),
            // Trans, // (9, Button::One),
            // Trans, // (29, Button::Two),
            // Trans, // (31, Button::Three),
            // Trans, // (13, Button::Four),
            // Trans, // (25, Button::Five),
            // Trans, // (27, Button::Six),
            // Trans, // (17, Button::Seven),
            // Trans, // (21, Button::Eight),
            // Trans, // (23, Button::Nine),
            // Trans, // (30, Button::VolumeUp),
            // Trans, // (10, Button::VolumeDown),
        ],
    ], &[ // Layer 2 (More Fun)
        // AM0
        &[k(Kb2),l(3),Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,k(F1),k(F2),Trans,Trans,Trans,Trans,Trans],
        // AM1
        &[Trans,k(F3),Trans,Trans,Trans,l(1),Trans,Trans,
            k(F4),k(F5),Trans,Trans,Trans,k(F6),Trans,Trans],
        // AM2
        &[Trans,Trans,Trans,Trans,Trans,HUNDRED,Trans,k(F7),
            Trans,k(F8),k(F9),Trans,Trans,Trans,Trans,Trans],
        // AM3
        &[Trans,Trans,Trans,Trans,l(5),Trans,Trans,OMEGA,
            k(F11),k(F12),Trans,Trans,Trans,Trans,Trans,Trans],
        // AM4
        &[k(End),k(PgUp),k(PgDown),k(Home),Trans,Trans,k(Delete),Trans,
            LEDS_DARKER, // Encoder clockwise
            LEDS_BRIGHTER, // Encoder counterclockwise
            LEDS_TOGGLE, // Encoder press
            Trans,Trans, // Unused pins (grounded)
            LEDS_BRIGHTER,LEDS_DARKER,LEDS_TOGGLE], // Macro1, Macro2, and Macro3 (respectively)
        // Everything past this point is just for infrared stuff
        // IR0
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
    ], &[ // Layer 3 (Fun-More Fun)
        // AM0
        &[k(Kb3),Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM1
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM2
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM3
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM4
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            DISP_DARKER, // Encoder clockwise
            DISP_BRIGHTER, // Encoder counterclockwise
            DISP_TOGGLE, // Encoder press
            Trans,Trans, // Unused pins (grounded)
            RESET,Trans,DISP_TOGGLE], // Macro1, Macro2, and Macro3 (respectively)
        // Everything past this point is just for infrared stuff
        // IR0
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
    ], &[ // Layer 4 (LAlt-Fun or LAlt-More Fun)
        // AM0
        &[k(Kb4),Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM1
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM2
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM3
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM4
        &[Trans,THUMBSUP,Trans,Trans,Trans,Trans,Trans,Trans,
            DISP_DARKER, // Encoder clockwise
            DISP_BRIGHTER, // Encoder counterclockwise
            DISP_TOGGLE, // Encoder press
            Trans,Trans, // Unused pins (grounded)
            RESET,Trans,DISP_TOGGLE], // Macro1, Macro2, and Macro3 (respectively)
        // Everything past this point is just for infrared stuff
        // IR0
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
    ], &[ // Layer 5 (RAlt-Fun or RAlt-More Fun)
        // AM0
        &[k(Kb5),Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM1
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM2
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM3
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM4
        &[Trans,THUMBSUP,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans, // Encoder clockwise
            Trans, // Encoder counterclockwise
            Trans, // Encoder press
            Trans,Trans, // Unused pins (grounded)
            Trans,Trans,Trans], // Macro1, Macro2, and Macro3 (respectively)
        // Everything past this point is just for infrared stuff
        // IR0
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
    ], &[ // Layer 6
        // AM0
        &[k(Kb6),Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM1
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM2
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM3
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
        // AM4
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans, // Encoder clockwise
            Trans, // Encoder counterclockwise
            Trans, // Encoder press
            Trans,Trans, // Unused pins (grounded)
            Trans,Trans,Trans], // Macro1, Macro2, and Macro3 (respectively)
        // Everything past this point is just for infrared stuff
        // IR0
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,
            Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans],
    ],
];

// TEMP:
const BIGMACRO: Action<CustomActions> = Sequence {
    events: &[ // Types out a macro
        Press(LShift),
        Tap(A),
        Release(LShift),
        Tap(B),
        Tap(C),
        // ...so on and so forth
        Complete,
    ],
};

/// Map key locations to multiplexer pins
pub type MuxMap = &'static [&'static [&'static [u8; 2]]];

pub static MUX_MAPPING: MuxMap = &[
    /*
    Since Keyberon was made for key switch matrices and not multi-channel analog multiplexers
    we need to map which key goes to which channel on which multiplexer.
    */
    &[ // Row 0 (just the ESC key)
        &[0,8],
    ],
    &[ // Row 1 (Tilda row)
        &[0,0], &[0,0], &[0,0], &[0,0], &[0,0], &[0,0], &[0,0], &[0,0], &[0,0], &[0,0], &[0,0],&[0,0],
    ],
    &[ // Row 2 (Tab row)
        &[0,0],
    ],
    &[ // Row 3 (CapsLock row)
        &[0,0],
    ],
    &[ // Row 4 (Shift row)
        &[0,0],
    ],
    &[ // Row 5 (Ctrl row)
        &[0,0],
    ],
    &[ // Infrared receiver (virtual row)
        &[0,0],
    ],
];