use common::types::{Current, ElectricalAngle};
use fixed::traits::ToFixed;

pub fn ipark(iq: Current, id: Current, angle: ElectricalAngle) -> (Current, Current) {
    let (sin, cos) = cordic::sin_cos(angle.to_fixed::<fixed::types::I11F21>());

    let sin = sin.to_fixed::<Current>();
    let cos = cos.to_fixed::<Current>();

    (cos * id - sin * iq, sin * id + cos * iq)
}
