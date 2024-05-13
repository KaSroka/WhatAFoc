use common::{
    types::{Angle, Dt, Speed, Torque}, ControllerCommand, ControllerMode, ControllerRunParams
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Receiver};
use fixed::traits::ToFixed;

use crate::{
    motor::traits::Motor,
    pid::{PController, PIController},
};

pub struct ControllerStatus {
    pub angle: Angle,
    pub speed: Speed,
    pub torque: Torque,
    pub loop_time_us: u64,
    pub target_reached: bool,
}

pub struct Controller<MOTOR> {
    motor: MOTOR,
    state: ControllerMode,
    last_torque: Torque,
    ctl: Receiver<'static, ThreadModeRawMutex, ControllerCommand, 20>,
    speed_pi: PIController,
    angle_p: PController,
    target_reached: bool,
}

impl<MOTOR> Controller<MOTOR>
where
    MOTOR: Motor,
{
    pub fn new(
        motor: MOTOR,
        ctl: Receiver<'static, ThreadModeRawMutex, ControllerCommand, 20>,
        dt: Dt,
    ) -> Self {
        let speed_pi = PIController::new(0.01.to_fixed(), 0.5.to_fixed(), dt);
        let angle_p = PController::new(500.unwrapped_to_fixed());

        Self {
            motor,
            state: ControllerMode::Idle,
            last_torque: Torque::ZERO,
            ctl,
            speed_pi,
            angle_p,
            target_reached: false,
        }
    }

    pub fn update(&mut self) {
        if let Ok(cmd) = self.ctl.try_receive() {
            match cmd {
                ControllerCommand::SetMode(new_state) => {
                    let reset = !matches!(
                        (&self.state, &new_state),
                        (ControllerMode::Idle, ControllerMode::Idle)
                            | (
                                ControllerMode::Run(ControllerRunParams::Torque { .. }),
                                ControllerMode::Run(ControllerRunParams::Torque { .. }),
                            )
                            | (
                                ControllerMode::Run(ControllerRunParams::Velocity { .. }),
                                ControllerMode::Run(ControllerRunParams::Velocity { .. }),
                            )| (
                                ControllerMode::Run(ControllerRunParams::Angle { .. }),
                                ControllerMode::Run(ControllerRunParams::Angle { .. }),
                            )
                    );

                    if reset {
                        self.speed_pi.reset();
                    }

                    self.target_reached = false;

                    self.state = new_state;
                }
            }
        }

        match &self.state {
            ControllerMode::Idle => {
                self.motor.idle().unwrap();
                self.target_reached = true;
            }
            ControllerMode::Run(params) => match params {
                ControllerRunParams::Torque { target } => {
                    self.motor.update(*target).unwrap();
                    self.last_torque = *target;

                    self.target_reached = true;
                }
                ControllerRunParams::Velocity {
                    target,
                    torque_limit,
                } => {
                    let speed_error = target - self.motor.speed();

                    let torque = self.speed_pi.update(speed_error);

                    if torque.abs() > torque_limit.abs() {
                        self.speed_pi.decay();
                    }

                    let torque = torque.clamp(-torque_limit.abs(), torque_limit.abs());

                    self.motor.update(torque).unwrap();
                    self.last_torque = torque;

                    let abs_error = speed_error.abs();

                    if abs_error < 1.to_fixed::<Speed>() {
                        self.target_reached = true;
                    } else if abs_error > 2.to_fixed::<Speed>() {
                        self.target_reached = false;
                    }
                }
                ControllerRunParams::Angle {
                    target,
                    speed_limit,
                    torque_limit,
                } => {
                    let angle_error = target - self.motor.angle();

                    let speed = self.angle_p.update(angle_error.saturating_to_fixed());
                    let speed = speed.clamp(-speed_limit.abs(), speed_limit.abs());

                    let torque = self.speed_pi.update(speed - self.motor.speed());
                    if torque.abs() > torque_limit.abs() {
                        self.speed_pi.decay();
                    }
                    let torque = torque.clamp(-torque_limit.abs(), torque_limit.abs());

                    self.motor.update(torque).unwrap();
                    self.last_torque = torque;

                    let abs_error = angle_error.abs();

                    if abs_error < 0.1.to_fixed::<Speed>() {
                        self.target_reached = true;
                    } else if abs_error > 0.2.to_fixed::<Speed>() {
                        self.target_reached = false;
                    }
                }
            },
            ControllerMode::CalibrateEncoder(_) => todo!(),
        }
    }

    pub fn motor(&self) -> &MOTOR {
        &self.motor
    }

    pub fn torque(&self) -> Torque {
        self.last_torque
    }

    pub fn target_reached(&self) -> bool {
        self.target_reached
    }
}
