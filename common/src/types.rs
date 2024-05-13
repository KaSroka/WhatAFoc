use fixed::types::{I11F21, I43F21, U1F31};

pub type NormalFixed = I11F21;
pub type ExtendedFixed = I43F21;

pub type RawAngle = NormalFixed;
pub type ElectricalAngle = NormalFixed;
pub type Speed = NormalFixed;
pub type Inductance = NormalFixed;
pub type Current = NormalFixed;
pub type Torque = NormalFixed;

pub type Angle = ExtendedFixed;
pub type ElectricalSpeed = ExtendedFixed;

pub type Dt = U1F31;
