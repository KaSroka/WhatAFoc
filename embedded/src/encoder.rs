use common::types::{Angle, Current, Dt, ElectricalAngle, ElectricalSpeed, RawAngle, Speed};
use defmt::{debug, Debug2Format};
use embassy_time::Timer;
use fixed::{traits::ToFixed, types::U32F0};

use crate::{lpf::Lpf, motor::traits::BaseMotor, pll::Pll};

use self::traits::BaseEncoder;

pub mod traits {
    use common::types::{Angle, Current, ElectricalAngle, ElectricalSpeed, RawAngle, Speed};
    use core::fmt::Debug;

    use crate::motor::traits::BaseMotor;

    pub trait BaseEncoder {
        type Error: Debug;

        const OVERFLOW_VAL: RawAngle;

        fn read(&mut self) -> Result<RawAngle, Self::Error>;
    }

    pub trait Encoder {
        type Error: Debug;

        fn update(&mut self) -> Result<(), Self::Error>;

        fn electrical_angle(&self) -> ElectricalAngle;
        fn angle(&self) -> Angle;

        fn electrical_speed(&self) -> ElectricalSpeed;
        fn speed(&self) -> Speed;

        async fn calibrate_electrical_angle<MOTOR>(
            &mut self,
            motor: &mut MOTOR,
            calibration_current: Current,
        ) -> Result<(), Self::Error>
        where
            MOTOR: BaseMotor;
    }
}

pub struct Encoder<BaseEnc> {
    inner: BaseEnc,
    angle: Angle,
    electrical_offset: ElectricalAngle,
    prev_raw_angle: RawAngle,
    prev_angle_interpolation: Angle,
    pole_pairs: u16,
    pub pll: Pll,
}

impl<BaseEnc> Encoder<BaseEnc>
where
    BaseEnc: BaseEncoder,
{
    pub fn new(base: BaseEnc, pole_pairs: u16, dt: Dt) -> Self {
        let pll = Pll::new(2000, dt);

        Self {
            inner: base,
            angle: Angle::ZERO,
            electrical_offset: ElectricalAngle::ZERO,
            prev_raw_angle: RawAngle::ZERO,
            prev_angle_interpolation: Angle::ZERO,
            pole_pairs,
            pll,
        }
    }

    fn electrical_angle_raw(&self) -> RawAngle {
        self.angle
            .unwrapped_mul_int(self.pole_pairs as i64)
            .frac()
            .to_fixed::<RawAngle>()
            - self.electrical_offset
    }

    pub fn inner(&self) -> &BaseEnc {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut BaseEnc {
        &mut self.inner
    }
}

impl<BaseEnc> traits::Encoder for Encoder<BaseEnc>
where
    BaseEnc: BaseEncoder,
{
    type Error = BaseEnc::Error;

    fn update(&mut self) -> Result<(), Self::Error> {
        let raw_angle = self.inner.read()?;
        let mut delta = raw_angle - self.prev_raw_angle;

        if delta > BaseEnc::OVERFLOW_VAL.wrapping_div_int(2) {
            delta -= BaseEnc::OVERFLOW_VAL;
        } else if delta < BaseEnc::OVERFLOW_VAL.wrapping_div_int(-2) {
            delta += BaseEnc::OVERFLOW_VAL;
        }

        let delta = delta.to_fixed::<Angle>();

        self.angle += delta - self.prev_angle_interpolation;

        self.pll.update(self.angle);

        let angle_interpolation = delta;

        self.angle += angle_interpolation;

        self.prev_raw_angle = raw_angle;
        self.prev_angle_interpolation = angle_interpolation;

        Ok(())
    }

    fn electrical_angle(&self) -> ElectricalAngle {
        (self.electrical_angle_raw() + 0.25.to_fixed::<ElectricalAngle>()).frac()
            * ElectricalAngle::TAU
    }

    fn angle(&self) -> Angle {
        self.angle * Angle::TAU
    }

    fn electrical_speed(&self) -> ElectricalSpeed {
        self.speed()
            .wide_mul_unsigned(self.pole_pairs.to_fixed::<U32F0>())
    }

    fn speed(&self) -> Speed {
        self.pll.estimate() * Speed::TAU
    }

    async fn calibrate_electrical_angle<MOTOR>(
        &mut self,
        motor: &mut MOTOR,
        calibration_current: Current,
    ) -> Result<(), Self::Error>
    where
        MOTOR: BaseMotor,
    {
        let mut lpf = Lpf::<10, Angle>::new();

        self.electrical_offset = ElectricalAngle::ZERO;

        motor
            .set_foc(calibration_current, Default::default(), Default::default())
            .unwrap();

        Timer::after_millis(500).await;

        for _ in 0..4096 {
            self.update()?;

            lpf.add(self.electrical_angle_raw().to_num());
        }

        motor.disable().unwrap();

        self.electrical_offset = lpf.val().to_num();

        debug!(
            "Encoder electrical offset: {}",
            Debug2Format(&self.electrical_offset)
        );

        Ok(())
    }
}
