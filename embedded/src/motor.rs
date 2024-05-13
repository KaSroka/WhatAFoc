use core::fmt::Debug;

use common::types::{Angle, Current, ElectricalAngle, ElectricalSpeed, Inductance, Speed, Torque};
use fixed::traits::ToFixed;

use crate::encoder;

pub mod traits {
    use core::fmt::Debug;

    use common::types::{Angle, Current, ElectricalAngle, ElectricalSpeed, Speed, Torque};

    use super::Inductance;
    pub trait BaseMotor {
        type Error: Debug;

        fn set_foc(
            &mut self,
            iq: Current,
            id: Current,
            angle: ElectricalAngle,
        ) -> Result<(), Self::Error>;

        fn disable(&mut self) -> Result<(), Self::Error>;
    }

    pub trait Motor {
        type Error: Debug;

        async fn calibrate_electrical_angle(
            &mut self,
            calibration_current: Current,
        ) -> Result<(), Self::Error>;

        fn set_foc(
            &mut self,
            iq: Current,
            id: Current,
            angle: ElectricalAngle,
        ) -> Result<(), Self::Error>;

        fn disable(&mut self) -> Result<(), Self::Error>;

        fn inductance(&self) -> Inductance;

        fn update(&mut self, torque: Torque) -> Result<(), Self::Error>;
        fn idle(&mut self) -> Result<(), Self::Error>;

        fn electrical_angle(&self) -> ElectricalAngle;
        fn angle(&self) -> Angle;

        fn electrical_speed(&self) -> ElectricalSpeed;
        fn speed(&self) -> Speed;
    }
}

#[derive(Debug)]
pub enum MotorWithEncoderError<MotorError: Debug, EncoderError: Debug> {
    EncoderError(EncoderError),
    MotorError(MotorError),
}

pub struct MotorWithEncoder<BaseMotor, Encoder> {
    inner: BaseMotor,
    encoder: Encoder,
    inductance: Inductance,
}

impl<BaseMotor, Encoder> MotorWithEncoder<BaseMotor, Encoder>
where
    BaseMotor: traits::BaseMotor,
    Encoder: encoder::traits::Encoder,
{
    pub fn new(inner: BaseMotor, encoder: Encoder, inductance: Inductance) -> Self {
        Self {
            inner,
            encoder,
            inductance,
        }
    }

    pub fn encoder(&self) -> &Encoder {
        &self.encoder
    }

    pub fn encoder_mut(&mut self) -> &mut Encoder {
        &mut self.encoder
    }
}

impl<BaseMotor, Encoder> traits::Motor for MotorWithEncoder<BaseMotor, Encoder>
where
    BaseMotor: traits::BaseMotor,
    Encoder: encoder::traits::Encoder,
{
    type Error = MotorWithEncoderError<BaseMotor::Error, Encoder::Error>;

    async fn calibrate_electrical_angle(
        &mut self,
        calibration_current: Current,
    ) -> Result<(), Self::Error> {
        self.encoder
            .calibrate_electrical_angle(&mut self.inner, calibration_current)
            .await
            .map_err(MotorWithEncoderError::EncoderError)
    }

    fn set_foc(
        &mut self,
        iq: Current,
        id: Current,
        angle: ElectricalAngle,
    ) -> Result<(), Self::Error> {
        self.inner
            .set_foc(iq, id, angle)
            .map_err(MotorWithEncoderError::MotorError)
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.inner
            .disable()
            .map_err(MotorWithEncoderError::MotorError)
    }

    fn inductance(&self) -> Inductance {
        self.inductance
    }

    fn update(&mut self, torque: Torque) -> Result<(), Self::Error> {
        self.encoder
            .update()
            .map_err(MotorWithEncoderError::EncoderError)?;

        let iq = torque;
        let id = -iq
            .saturating_mul(
                (self
                    .electrical_speed()
                    .saturating_mul(self.inductance().to_fixed()))
                .saturating_to_fixed::<Current>(),
            )
            .clamp(-torque.abs(), torque.abs());

        // let norm = (iq * iq + id * id).sqrt();

        // if norm != 0 {
        //     let norm = Current::ONE / norm * iq.abs();
        //     iq *= norm;
        //     id *= norm;
        // }

        self.inner
            .set_foc(iq, id, self.electrical_angle())
            .map_err(MotorWithEncoderError::MotorError)
    }

    fn idle(&mut self) -> Result<(), Self::Error> {
        self.inner
            .disable()
            .map_err(MotorWithEncoderError::MotorError)?;
        self.encoder
            .update()
            .map_err(MotorWithEncoderError::EncoderError)
    }

    fn electrical_angle(&self) -> ElectricalAngle {
        self.encoder.electrical_angle()
    }

    fn angle(&self) -> Angle {
        self.encoder.angle()
    }

    fn electrical_speed(&self) -> ElectricalSpeed {
        self.encoder.electrical_speed()
    }

    fn speed(&self) -> Speed {
        self.encoder.speed()
    }
}
