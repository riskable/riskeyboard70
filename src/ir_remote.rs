use infrared::{
    cmd::AddressCommand,
    protocol::{Nec, NecCommand},
    protocol::{Rc5, Rc5Command},
    protocol::{Rc6, Rc6Command},
    remotecontrol::{Action, Button, DeviceType, RemoteControlModel},
    ProtocolId,
};
use numtoa::NumToA;

static mut BUF: [u8; 20] = [0; 20];

#[derive(Debug, Default, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IRRemote;

impl IRRemote {
    pub fn index(button: Action) -> usize {
        for (i, b) in IRRemote::BUTTONS.iter().enumerate() {
            if b.1 == button {
                return i;
            }
        }
        0
    }

    pub fn to_string(code: u32) -> &'static str {
        unsafe {
            code.numtoa_str(10, &mut BUF)
        }
    }
}

// TODO: Figure out a way for end users to configure these values in the config
impl RemoteControlModel for IRRemote {
    const MODEL: &'static str = "Riskeyboard Remote";
    const DEVTYPE: DeviceType = DeviceType::TV;
    const PROTOCOL: ProtocolId = ProtocolId::Nec;
    const ADDRESS: u32 = 0;
    type Cmd = NecCommand;
    const BUTTONS: &'static [(u32, Action)] = &[
        // Cmdid to Button mappings
        (69, Action::Power),
        (71, Action::Menu),
        (68, Action::Info), // "TEST" button
        (64, Action::VolumeUp),
        (25, Action::VolumeDown),
        (13, Action::ChannelList), // "C" button
        (9, Action::Next),
        (7, Action::Prev),
        (21, Action::Play),
        (67, Action::Rewind), // Looks more like an undo button but whatever :shrug:
        (22, Action::Zero),
        (12, Action::One),
        (24, Action::Two),
        (94, Action::Three),
        (8, Action::Four), // Can't go beyond this (16 buttons) for now
        (28, Action::Five), // Need to figure out a better way to handle the mapping
        (90, Action::Six),
        (66, Action::Seven),
        (82, Action::Eight),
        (74, Action::Nine),
    ];

    fn decode(cmd: &Self::Cmd) -> Option<Action> {
        // Check address
        if Self::ADDRESS != cmd.address() {
            return None;
        }
        Self::BUTTONS
            .iter()
            .find(|(c, _)| *c == cmd.command())
            .map(|(_, b)| *b)
    }

    fn encode(button: &Action) -> Option<Self::Cmd> {
        Self::BUTTONS
            .iter()
            .find(|(_, b)| b == button)
            .and_then(|(c, _)| Self::Cmd::create(Self::ADDRESS, *c as u32))
    }
}

// Original remote:
// impl RemoteControlModel for IRRemote {
//     const MODEL: &'static str = "Riskeyboard Remote";
//     const DEVTYPE: DeviceType = DeviceType::TV;
//     const PROTOCOL: ProtocolId = ProtocolId::Nec;
//     const ADDRESS: u32 = 0;
//     type Cmd = NecCommand;
//     const BUTTONS: &'static [(u32, Action)] = &[
//         // Cmdid to Button mappings
//         (77, Action::Power),
//         (84, Action::Source),
//         (30, Action::VolumeUp),
//         (10, Action::VolumeDown),
//         (5, Action::Next),
//         (2, Action::Prev),
//         (22, Action::Mute),
//         (76, Action::Rewind),      // "Record"
//         (64, Action::PictureSize), // "Full Screen"
//         (12, Action::Time),        // "Time Shift"
//         (28, Action::Return),      // "Recall"
//         (18, Action::Zero),
//         (9, Action::One),
//         (29, Action::Two),
//         (31, Action::Three),
//         (13, Action::Four), // Can't go beyond this (16 buttons) for now
//         (25, Action::Five), // Need to figure out a better way to handle the mapping
//         (27, Action::Six),
//         (17, Action::Seven),
//         (21, Action::Eight),
//         (23, Action::Nine),
//     ];

//     fn decode(cmd: &Self::Cmd) -> Option<Action> {
//         // Check address
//         if Self::ADDRESS != cmd.address() {
//             return None;
//         }
//         Self::BUTTONS
//             .iter()
//             .find(|(c, _)| *c == cmd.command())
//             .map(|(_, b)| *b)
//     }

//     fn encode(button: &Action) -> Option<Self::Cmd> {
//         Self::BUTTONS
//             .iter()
//             .find(|(_, b)| b == button)
//             .and_then(|(c, _)| Self::Cmd::create(Self::ADDRESS, *c as u32))
//     }
// }