use infrared::{
    protocols::rc5::{Rc5, Rc5Command},
    protocols::rc6::{Rc6, Rc6Command},
    protocols::nec::{Nec, NecCommand},
    remotecontrol::{Button, DeviceType, RemoteControl},
    ProtocolId,
};

pub struct IRRemote;

impl IRRemote {
    pub fn index(button: Button) -> usize {
        for (i, b) in IRRemote::BUTTONS.iter().enumerate() {
            if b.1 == button {
                return i
            }
        }
        0
    }
}

impl RemoteControl for IRRemote {
    const MODEL: &'static str = "Riskeyboard Remote";
    const DEVTYPE: DeviceType = DeviceType::TV;
    const PROTOCOL: ProtocolId = ProtocolId::Nec;
    const ADDRESS: u32 = 0;
    type Cmd = NecCommand;
    const BUTTONS: &'static [(u32, Button)] = &[
        // Cmdid to Button mappings
        (77, Button::Power),
        (84, Button::Source),
        (30, Button::VolumeUp),
        (10, Button::VolumeDown),
        (5, Button::Next),
        (2, Button::Prev),
        (22, Button::Mute),
        (76, Button::Rewind), // "Record"
        (64, Button::PictureSize), // "Full Screen"
        (12, Button::Time), // "Time Shift"
        (28, Button::Return), // "Recall"
        (18, Button::Zero),
        (9, Button::One),
        (29, Button::Two),
        (31, Button::Three),
        (13, Button::Four), // Can't go beyond this (16 buttons) for now
        (25, Button::Five), // Need to figure out a better way to handle the mapping
        (27, Button::Six),
        (17, Button::Seven),
        (21, Button::Eight),
        (23, Button::Nine),
    ];
}