use defmt::{info, Debug2Format};
use embassy_time::Timer;
use fixed::{
    traits::ToFixed,
    types::I32F0,
};
use common::types::{Current, RawAngle};

use crate::{encoder::traits::BaseEncoder, lpf::Lpf, motor::traits::BaseMotor};


pub struct CalibratedEncoder<BASE> {
    inner: BASE,
    offset_table: [RawAngle; 100],
}

impl<BaseEnc> CalibratedEncoder<BaseEnc>
where
BaseEnc: BaseEncoder,
{
    pub fn new(inner: BaseEnc) -> Self {
        Self {
            inner,
            offset_table: [Default::default(); 100],
        }
    }

    #[allow(dead_code)]
    pub fn inner(&self) -> &BaseEnc {
        &self.inner
    }

    #[allow(dead_code)]
    pub fn inner_mut(&mut self) -> &mut BaseEnc {
        &mut self.inner
    }

    pub fn offsets(&self) -> &[RawAngle] {
        &self.offset_table
    }

    pub async fn calibrate_offsets<MOTOR>(
        &mut self,
        motor: &mut MOTOR,
        calibration_current: Current,
    ) -> Result<(), BaseEnc::Error>
    where
        MOTOR: BaseMotor,
    {
        let mut angle = RawAngle::ZERO;
        let delta = 0.5.to_fixed::<RawAngle>() / 64;
        let mut lpf = Lpf::<8, _>::new();

        for _ in 0..100 {
            for _ in 0..64 {
                motor
                .set_foc(
                    calibration_current >> 2,
                    Default::default(),
                    angle.frac() * RawAngle::TAU,
                )
                .unwrap();

                Timer::after_micros(500).await;
                angle += delta;
            }

            motor
            .set_foc(
                calibration_current,
                Default::default(),
                angle.frac() * RawAngle::TAU,
            )
            .unwrap();

            lpf.reset();

            let mut last_read = self.inner.read()?;

            for _ in 0..4096 {
                let current = self.inner.read()?;
                let mut delta = current - last_read;

                if delta > BaseEnc::OVERFLOW_VAL.wrapping_div_int(2) {
                    delta -= BaseEnc::OVERFLOW_VAL;
                } else if delta < BaseEnc::OVERFLOW_VAL.wrapping_div_int(-2) {
                    delta += BaseEnc::OVERFLOW_VAL;
                }

                lpf.add(current + delta);

                last_read = current;
            }

            let mut int_value: i32 = lpf.val().wide_mul(100.to_fixed::<I32F0>()).to_num();
            int_value += 49;

            if int_value < 0 {
                int_value += 100;
            } else if int_value > 99 {
                int_value -= 100;
            }

            let expected_value = int_value.to_fixed::<RawAngle>().unwrapped_div_int(100);

            let mut delta = lpf.val() - expected_value + 0.5.to_fixed::<RawAngle>();

            if delta > BaseEnc::OVERFLOW_VAL.wrapping_div_int(2) {
                delta -= BaseEnc::OVERFLOW_VAL;
            } else if delta < BaseEnc::OVERFLOW_VAL.wrapping_div_int(-2) {
                delta += BaseEnc::OVERFLOW_VAL;
            }

            info!("Angle: {}, delta: {}, expected: {}, val: {}", int_value, Debug2Format(&delta), Debug2Format(&expected_value), Debug2Format(&lpf.val()));

            self.offset_table[int_value as usize] += delta;
        }

        motor.disable().unwrap();

        let avg = self.offset_table.iter().sum::<RawAngle>() / self.offset_table.len().to_fixed::<RawAngle>();

        for offset in self.offset_table.iter_mut() {
            *offset -= avg;
        }

        Ok(())
    }
}

impl<BASE> BaseEncoder for CalibratedEncoder<BASE>
where
    BASE: BaseEncoder,
{
    type Error = BASE::Error;

    const OVERFLOW_VAL: RawAngle = BASE::OVERFLOW_VAL;

    fn read(&mut self) -> Result<RawAngle, Self::Error> {
        let angle = self.inner.read()?;

        let multipled_angle = angle.wide_mul(100.to_fixed::<I32F0>());

        let mut idx: i32 = multipled_angle.to_num();
        idx += 49;

        if idx < 0 {
            idx += 100;
        } else if idx > 99 {
            idx -= 100;
        }

        let idx = idx as usize;
        let idx_n = (idx + 1) % 100;
        let lerp = multipled_angle.frac().to_num::<RawAngle>();

        let corr = lerp.lerp(self.offset_table[idx], self.offset_table[idx_n]);


        Ok(angle - corr)
    }
}
