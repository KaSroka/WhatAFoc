use common::types::{Current, ElectricalAngle};
use embassy_stm32::gpio::{AnyPin, Level, Output};
use embedded_hal::Pwm;
use fixed::traits::ToFixed;

use crate::motor::traits::BaseMotor;

pub struct StepperMotor4S2P<PWM: Pwm> {
    out1_pos: Output<'static, AnyPin>,
    out1_neg: Output<'static, AnyPin>,
    out2_pos: Output<'static, AnyPin>,
    out2_neg: Output<'static, AnyPin>,
    pwm: PWM,
    pwm_max: PWM::Duty,
    out1_pwm_channel: PWM::Channel,
    out2_pwm_channel: PWM::Channel,
}

impl<PWM> StepperMotor4S2P<PWM>
where
    PWM: Pwm,
{
    pub fn new(
        out1_pos: Output<'static, AnyPin>,
        out1_neg: Output<'static, AnyPin>,
        out2_pos: Output<'static, AnyPin>,
        out2_neg: Output<'static, AnyPin>,
        pwm: PWM,
        out1_pwm_channel: PWM::Channel,
        out2_pwm_channel: PWM::Channel,
    ) -> Self {
        let pwm_max = pwm.get_max_duty();

        Self {
            out1_pos,
            out1_neg,
            out2_pos,
            out2_neg,
            pwm,
            pwm_max,
            out1_pwm_channel,
            out2_pwm_channel,
        }
    }
}

impl<PWM> BaseMotor for StepperMotor4S2P<PWM>
where
    PWM: Pwm<Duty = u16>,
    PWM::Channel: Copy,
{
    type Error = ();

    fn set_foc(
        &mut self,
        iq: Current,
        id: Current,
        angle: ElectricalAngle,
    ) -> Result<(), Self::Error> {
        let (ua, ub) = crate::utils::ipark(iq, id, angle);

        // info!(
        //     "Angle: {}, Iq: {}, Id: {}, Ua: {}, Ub: {}",
        //     Debug2Format(&angle),
        //     Debug2Format(&iq),
        //     Debug2Format(&id),
        //     Debug2Format(&ua),
        //     Debug2Format(&ub)
        // );

        let ua_dir = ua.is_negative();
        let ub_dir = ub.is_negative();
        let ua = ua
            .unsigned_abs()
            .clamp(0.to_fixed(), 1.to_fixed())
            .saturating_mul_int(self.pwm_max as u32)
            .to_num::<u16>();
        let ub = ub
            .unsigned_abs()
            .clamp(0.to_fixed(), 1.to_fixed())
            .saturating_mul_int(self.pwm_max as u32)
            .to_num::<u16>();
        // info!("Phase 1 PWM: {}/{}, dir: {}", ua, self.pwm_max, ua_dir);
        // info!("Phase 2 PWM: {}/{}, dir: {}", ub, self.pwm_max, ub_dir);

        self.pwm.set_duty(self.out1_pwm_channel, ua);
        self.pwm.set_duty(self.out2_pwm_channel, ub);

        self.out1_pos.set_level(ua_dir.into());
        self.out1_neg.set_level((!ua_dir).into());

        self.out2_pos.set_level(ub_dir.into());
        self.out2_neg.set_level((!ub_dir).into());

        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.pwm.set_duty(self.out1_pwm_channel, 0);
        self.pwm.set_duty(self.out2_pwm_channel, 0);

        self.out1_pos.set_level(Level::Low);
        self.out1_neg.set_level(Level::Low);

        self.out2_pos.set_level(Level::Low);
        self.out2_neg.set_level(Level::Low);

        Ok(())
    }
}
