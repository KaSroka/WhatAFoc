use defmt::{info, Debug2Format};
use fixed::{traits::ToFixed, types::{I11F21, I43F21, U11F21, U14F2, U16F0, U1F31}};

pub struct Pll {
    pub frequency: I11F21,
    pub phase: I43F21,
    dt: U1F31,
    kp_dt: U11F21,
    ki_dt: U11F21,
    snap_threshold: U11F21,
}

impl Pll {
    pub fn new(bw: u16, dt: U1F31) -> Self {
        let kp = bw.unwrapped_to_fixed::<U16F0>().wide_mul(2.unwrapped_to_fixed::<U14F2>());
        let ki = kp.unwrapped_mul(0.25.unwrapped_to_fixed()).unwrapped_mul(kp);
        let kp_dt = kp.wide_mul(dt).unwrapped_to_fixed::<U11F21>();
        let ki_dt = ki.wide_mul(dt).unwrapped_to_fixed::<U11F21>();
        let snap_threshold = ki_dt.unwrapped_div_int(16000);

        info!(
            "PLL params: kp {}, ki {}, kp_dt: {}, ki_dt: {}, snap_threshold: {}",
            Debug2Format(&kp),
            Debug2Format(&ki),
            Debug2Format(&kp_dt),
            Debug2Format(&ki_dt),
            Debug2Format(&snap_threshold),
        );

        Self {
            frequency: I11F21::ZERO,
            phase: I43F21::ZERO,
            dt,
            kp_dt,
            ki_dt,
            snap_threshold,
        }
    }

    pub fn update(&mut self, actual_integral: I43F21) {
        let step = self.frequency.wide_mul_unsigned(self.dt).unwrapped_to_fixed::<I11F21>();
        self.phase += step.to_num::<I43F21>();
        let delta = (actual_integral.to_num::<I43F21>() - self.phase).to_num::<I11F21>();
        // info!("PLL update actual value :{}, current integral: {}, delta: {}, current base: {}, step: {}", Display2Format(&actual_integral), Display2Format(&self.integral), Display2Format(&delta), Display2Format(&self.base), Display2Format(&step));
        self.phase += self.kp_dt.wide_mul_signed(delta).unwrapped_to_fixed::<I43F21>();
        self.frequency += self.ki_dt.wide_mul_signed(delta).unwrapped_to_fixed::<I11F21>();

        // info!("PLL update adjusted integral: {}, adjusted base: {}", Display2Format(&self.integral), Display2Format(&self.base));

        if self.frequency.abs() < self.snap_threshold {
            self.frequency = I11F21::ZERO;
        }
    }

    pub fn estimate(&self) -> I11F21 {
        self.frequency
    }
}
