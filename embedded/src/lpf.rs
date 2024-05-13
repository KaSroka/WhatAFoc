use fixed::traits::Fixed;

pub struct Lpf<const TF: u32, F> {
    val: F,
}

impl<const TF: u32, F> Lpf<TF, F>
where
    F: Fixed,
{
    pub fn new() -> Self {
        Self { val: F::ZERO }
    }

    pub fn val(&self) -> F {
        self.val >> TF
    }

    pub fn add(&mut self, val: F) -> F {
        self.val += val - (self.val >> TF);

        self.val >> TF
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.val = F::ZERO;
    }
}
