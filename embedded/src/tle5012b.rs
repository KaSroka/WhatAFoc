use core::fmt::Debug;

use crate::encoder::traits::BaseEncoder;
use common::types::RawAngle;
use embassy_stm32::gpio::{AnyPin, Output};
use fixed::types::I17F15;

pub struct TLE5012B<SPI> {
    _spi: SPI,
    cs: Output<'static, AnyPin>,
}

impl<SPI> TLE5012B<SPI>
where
    SPI: embedded_hal::blocking::spi::Transfer<u16>,
{
    #[allow(clippy::unusual_byte_groupings)]
    const ANGLE_REV_REQUEST: u16 = 0b1_0000_0_000010_0000;
    // const CALIB_REQUEST: [u16; 2] = [0b0_1010_0_001000_0000, 0b0_000_1000_0000_0_0_01];

    pub async fn new(spi: SPI, cs: Output<'static, AnyPin>) -> Result<Self, SPI::Error> {
        let encoder = Self { _spi: spi, cs };

        embassy_stm32::pac::SPI1
            .cr1()
            .modify(|w| w.set_dff(embassy_stm32::pac::spi::vals::Dff::SIXTEENBIT));
        embassy_stm32::pac::SPI1.cr1().modify(|w| w.set_spe(true));

        Ok(encoder)
    }
}

impl<SPI> BaseEncoder for TLE5012B<SPI>
where
    SPI: embedded_hal::blocking::spi::Transfer<u16>,
    SPI::Error: Debug,
{
    type Error = SPI::Error;

    const OVERFLOW_VAL: RawAngle = RawAngle::ONE;

    fn read(&mut self) -> Result<RawAngle, Self::Error> {
        self.cs.set_low();

        while !embassy_stm32::pac::SPI1.sr().read().txe() {}
        embassy_stm32::pac::SPI1
            .dr()
            .write(|val| val.set_dr(Self::ANGLE_REV_REQUEST));
        while !embassy_stm32::pac::SPI1.sr().read().rxne() {}
        embassy_stm32::pac::SPI1.dr().read();

        embassy_stm32::pac::GPIOA.cr(0).modify(|w| {
            w.set_mode(7, embassy_stm32::pac::gpio::vals::Mode::INPUT);
            w.set_cnf_in(7, embassy_stm32::pac::gpio::vals::CnfIn::FLOATING);
        });

        while !embassy_stm32::pac::SPI1.sr().read().txe() {}
        embassy_stm32::pac::SPI1
            .dr()
            .write(|val| val.set_dr(0xffff));
        while !embassy_stm32::pac::SPI1.sr().read().rxne() {}
        let resp = embassy_stm32::pac::SPI1.dr().read().dr();

        self.cs.set_high();

        embassy_stm32::pac::GPIOA.cr(0).modify(|w| {
            w.set_mode(7, embassy_stm32::pac::gpio::vals::Mode::OUTPUT50MHZ);
            w.set_cnf_out(7, embassy_stm32::pac::gpio::vals::CnfOut::ALTPUSHPULL);
        });

        let raw_angle = ((resp as i16) << 1) >> 1;
        Ok(I17F15::from_bits(raw_angle as i32).to_num())
    }
}
