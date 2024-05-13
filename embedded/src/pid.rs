use common::types::{Dt, NormalFixed};
use fixed::{traits::ToFixed, types::I0F32};

pub struct PController {
    kp: NormalFixed,
}

impl PController {
    pub fn new(kp: NormalFixed) -> Self {
        Self { kp }
    }

    pub fn update(&self, error: NormalFixed) -> NormalFixed {
        self.kp.saturating_mul(error)
    }
}

pub struct IController {
    ki_dt: I0F32,
    integral: NormalFixed,
}

impl IController {
    pub fn new(ki: NormalFixed, dt: Dt) -> Self {
        let ki_dt = ki.wide_mul_unsigned(dt).unwrapped_to_fixed();
        Self { ki_dt, integral: NormalFixed::ZERO }
    }

    pub fn update(&mut self, error: NormalFixed) -> NormalFixed {
        self.integral = self.integral.saturating_add(self.ki_dt.wide_mul(error).saturating_to_fixed());
        self.integral
    }

    pub fn decay(&mut self) {
        self.integral *= 0.99.to_fixed::<NormalFixed>();
    }

    pub fn reset(&mut self) {
        self.integral = NormalFixed::ZERO;
    }
}

pub struct PIController {
    p: PController,
    i: IController,
}

impl PIController {
    pub fn new(kp: NormalFixed, ki: NormalFixed, dt:Dt) -> Self {
        let p = PController::new(kp);
        let i = IController::new(ki, dt);

        Self {p, i}
    }

    pub fn update(&mut self, error: NormalFixed) -> NormalFixed {
        self.p.update(error).saturating_add(self.i.update(error))
    }

    pub fn decay(&mut self) {
        self.i.decay();
    }

    pub fn reset(&mut self) {
        self.i.reset();
    }
}
