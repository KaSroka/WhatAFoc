#![cfg_attr(feature = "embedded", no_std)]

use serde::{Deserialize, Serialize};
use types::{Angle, RawAngle, Speed, Torque};

pub mod types;

pub type Error = postcard::Error;

#[cfg_attr(feature = "embedded", derive(Serialize))]
#[cfg_attr(not(feature = "embedded"), derive(Deserialize))]
#[derive(Debug, Clone, Copy)]
pub struct EmbeddedState {
    pub loop_time_us: u64,
    pub torque: Torque,
    pub speed: Speed,
    pub angle: Angle,
    pub target_reached: bool,
}

#[cfg_attr(feature = "embedded", derive(Debug, Clone, Serialize))]
#[cfg(feature = "embedded")]
pub enum EmbeddedMessage<'m> {
    State(EmbeddedState),
    OffsetTable(&'m [RawAngle]),
}

#[cfg_attr(not(feature = "embedded"), derive(Debug, Clone, Deserialize))]
#[cfg(not(feature = "embedded"))]
pub enum EmbeddedMessage {
    State(EmbeddedState),
    OffsetTable(Vec<RawAngle>),
}

#[cfg_attr(not(feature = "embedded"), derive(Serialize))]
#[cfg_attr(feature = "embedded", derive(Deserialize))]
#[derive(Debug, Clone, Copy)]
pub enum ControllerRunParams {
    Torque {
        target: Torque,
    },
    Velocity {
        target: Speed,
        torque_limit: Torque,
    },
    Angle {
        target: Angle,
        speed_limit: Speed,
        torque_limit: Torque,
    },
}

#[cfg_attr(not(feature = "embedded"), derive(Serialize))]
#[cfg_attr(feature = "embedded", derive(Deserialize))]
#[derive(Debug, Clone, Copy)]
pub enum ControllerCalibrateEncoderStatus {
    Init,
    Progress(usize),
}

#[cfg_attr(not(feature = "embedded"), derive(Serialize))]
#[cfg_attr(feature = "embedded", derive(Deserialize))]
#[derive(Debug, Clone, Copy)]
pub enum ControllerMode {
    Idle,
    Run(ControllerRunParams),
    CalibrateEncoder(ControllerCalibrateEncoderStatus),
}

#[cfg_attr(not(feature = "embedded"), derive(Serialize))]
#[cfg_attr(feature = "embedded", derive(Deserialize))]
#[derive(Debug, Clone)]
pub enum ControllerCommand {
    SetMode(ControllerMode),
}

#[cfg(not(feature = "embedded"))]
pub fn encode_host_message(msg: &ControllerCommand) -> Result<Vec<u8>, postcard::Error> {
    postcard::to_stdvec_cobs(&msg)
}

#[cfg(feature = "embedded")]
pub fn decode_host_message(data: &mut [u8]) -> Result<ControllerCommand, postcard::Error> {
    postcard::from_bytes_cobs(data)
}

#[cfg(feature = "embedded")]
pub fn encode_embedded_message<'b>(
    msg: &EmbeddedMessage,
    buffer: &'b mut [u8],
) -> Result<&'b mut [u8], postcard::Error> {
    postcard::to_slice_cobs(msg, buffer)
}

#[cfg(not(feature = "embedded"))]
pub fn decode_embedded_message(data: &mut [u8]) -> Result<EmbeddedMessage, postcard::Error> {
    postcard::from_bytes_cobs(data)
}

// pub fn add(left: usize, right: usize) -> usize {
//     left + right
// }

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }
