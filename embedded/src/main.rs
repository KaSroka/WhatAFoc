#![no_std]
#![no_main]

use common::types::Dt;
use common::{EmbeddedMessage, EmbeddedState};
use defmt::{error, info, unwrap, Debug2Format};
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{AnyPin, Level, Output, OutputType, Speed};
use embassy_stm32::peripherals::{DMA1_CH4, DMA1_CH5, USART1};
use embassy_stm32::spi::{self, BitOrder, Spi};
use embassy_stm32::timer::{simple_pwm, Channel};
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_stm32::{time::Hertz, Config};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal::spi::{Phase, Polarity};
use fixed::traits::ToFixed;
use fixed::types::U32F0;

use crate::controller::ControllerStatus;
use crate::motor::traits::Motor;

use {defmt_rtt as _, panic_probe as _};

mod calibrated_encoder;
mod controller;
mod encoder;
mod lpf;
mod motor;
mod pll;
mod stepper_motor;
mod tle5012b;
mod utils;
mod pid;

bind_interrupts!(struct Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});

static CONTROLLER_CTL_QUEUE: embassy_sync::channel::Channel<
    ThreadModeRawMutex,
    common::ControllerCommand,
    20,
> = embassy_sync::channel::Channel::new();
static CONTROLLER_STATUS: embassy_sync::signal::Signal<
    ThreadModeRawMutex,
    controller::ControllerStatus,
> = embassy_sync::signal::Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz(8_000_000));
    config.rcc.sys_ck = Some(Hertz(72_000_000));
    config.rcc.pclk1 = Some(Hertz(36_000_000));
    config.rcc.pclk2 = Some(Hertz(72_000_000));

    let p = embassy_stm32::init(config);

    embassy_stm32::pac::AFIO.mapr().modify(|w| {
        w.set_swj_cfg(0b001);
        w.set_tim3_remap(0b10);
    });

    let led = Output::new(p.PC13, Level::Low, Speed::VeryHigh).degrade();
    unwrap!(spawner.spawn(led_blinker(led, Duration::from_millis(500))));

    info!("Starting SPI configuration");

    let mut spi_config = spi::Config::default();
    spi_config.mode.polarity = Polarity::IdleLow;
    spi_config.mode.phase = Phase::CaptureOnSecondTransition;
    spi_config.frequency = Hertz(8_000_000);
    spi_config.bit_order = BitOrder::MsbFirst;

    let tle_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi = Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config,
    );

    info!("Starting PWM configuration");

    let ch1 = simple_pwm::PwmPin::new_ch1(p.PB4, OutputType::PushPull);
    let ch2 = simple_pwm::PwmPin::new_ch2(p.PB5, OutputType::PushPull);

    let mut pwm = simple_pwm::SimplePwm::new(
        p.TIM3,
        Some(ch1),
        Some(ch2),
        None,
        None,
        Hertz(281_250),
        Default::default(),
    );

    let config = Default::default();
    let usart = embassy_stm32::usart::Uart::new(
        p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, config,
    )
    .unwrap();

    gd32_frequency_boost();

    pwm.enable(Channel::Ch1);
    pwm.enable(Channel::Ch2);

    let pwm_max = pwm.get_max_duty() - 1;

    info!("Max PWM value {}", pwm_max);

    pwm.set_duty(Channel::Ch1, 0);
    pwm.set_duty(Channel::Ch2, 0);

    let mot_out1_neg = Output::new(p.PB8, Level::High, Speed::VeryHigh).degrade();
    let mot_out1_pos = Output::new(p.PB9, Level::High, Speed::VeryHigh).degrade();
    let mot_out2_neg = Output::new(p.PB6, Level::High, Speed::VeryHigh).degrade();
    let mot_out2_pos = Output::new(p.PB7, Level::High, Speed::VeryHigh).degrade();

    // base_encoder
    //     .calibrate_offsets(|angle| {
    //         info!("Angle: {}", Debug2Format(&angle));
    //         motor
    //             .set_foc(0.5.to_fixed(), Default::default(), angle.to_num())
    //             .unwrap();
    //     })
    //     .await
    //     .unwrap();

    // let mut arr = [fixed::types::I17F15::ZERO; 400];
    // const MOTOR_POLE_PAIRS: fixed::types::I17F15 = fixed::types::I17F15::lit("50");

    // for (i, elem) in arr.iter_mut().enumerate() {
    //     let angle = fixed::types::I17F15::FRAC_PI_4 * i.to_fixed::<fixed::types::I17F15>();
    //     move_motor(angle, 1.to_fixed(), 0.to_fixed());
    //     Timer::after_millis(50).await;
    //     let mut lpf = Lpf::new(3);
    //     for _ in 0..128 {
    //         let enc_angle = base_encoder.read().await.unwrap();
    //         lpf.add(enc_angle);
    //     }
    //     let angle = angle / MOTOR_POLE_PAIRS;
    //     let mut delta = lpf.val().to_num::<fixed::types::I17F15>() * fixed::types::I17F15::PI * 2.to_fixed::<fixed::types::I17F15>() - angle;

    //     if delta < fixed::types::I17F15::PI {
    //         delta += fixed::types::I17F15::PI + fixed::types::I17F15::PI;
    //     }

    //     if delta < fixed::types::I17F15::ZERO {
    //         delta += fixed::types::I17F15::PI + fixed::types::I17F15::PI;
    //     }

    //     *elem = delta;

    //     info!("Encoder delta at angle {}: {}, encoder angle: {}", Display2Format(&angle), Display2Format(&delta), Display2Format(&lpf.val()));
    // }

    // info!("Encoder deltas: {}", Debug2Format(&arr));

    let loop_time = Duration::from_hz(20000);
    let dt = loop_time.as_micros().to_fixed::<U32F0>().wide_div(1000000.to_fixed::<U32F0>()).to_fixed::<Dt>();

    let encoder = tle5012b::TLE5012B::new(spi, tle_cs.degrade())
        .await
        .unwrap();


    let mut motor = stepper_motor::StepperMotor4S2P::new(
        mot_out1_pos,
        mot_out1_neg,
        mot_out2_pos,
        mot_out2_neg,
        pwm,
        Channel::Ch1,
        Channel::Ch2,
    );

    let mut encoder = calibrated_encoder::CalibratedEncoder::new(encoder);

    encoder.calibrate_offsets(&mut motor, 0.5.to_fixed()).await.unwrap();

    let encoder = encoder::Encoder::new(encoder, 50, dt);

    let mut motor =
        motor::MotorWithEncoder::new(motor, encoder, fixed::types::I11F21::lit("0.0028"));

    let (tx, rx) = usart.split();

    unwrap!(spawner.spawn(uart_parser(rx)));
    unwrap!(spawner.spawn(uart_sender(tx)));

    motor
        .calibrate_electrical_angle(1.to_fixed())
        .await
        .unwrap();

    // let target: fixed::types::I17F15 = 0.12.to_fixed();
    // let mut last_angle = encoder.angle();



    let mut controller =
        controller::Controller::new(motor, CONTROLLER_CTL_QUEUE.receiver(), dt);

    let mut ticker = Ticker::every(loop_time);



    loop {
        ticker.next().await;
        let start = Instant::now();
        controller.update();

        let loop_time = Instant::now().saturating_duration_since(start);

        CONTROLLER_STATUS.signal(ControllerStatus {
            angle: controller.motor().angle(),
            speed: controller.motor().speed(),
            torque: controller.torque(),
            loop_time_us: loop_time.as_micros(),
            target_reached: controller.target_reached(),
        });
    }

    // loop {
    //     let now = Instant::now();

    //     for _ in 0..1_000 {
    //         motor.update().unwrap();

    //         ticker.next().await;
    //         // info!(
    //         //     "{} {}",
    //         //     Debug2Format(&encoder.angle()),
    //         //     Debug2Format(&encoder.electrical_angle())
    //         // );
    //     }

    //     let duration = now.elapsed();
    //     let single_duration_us = duration.as_micros() / 1_000;
    //     let single_duration = duration.as_micros() as f32 / 1_000f32 / 1_000_000f32;
    //     let total_duration = duration.as_micros() as f32 / 1_000_000f32;
    //     let curr_angle = encoder.angle();

    //     let new_state = EmbeddedState {
    //         loop_time: single_duration,
    //         speed: encoder.speed(),
    //     };

    //     if let Ok(mut state) = STATE.try_lock() {
    //         *state = new_state;
    //     }

    //     last_angle = curr_angle;
    // }
}

#[embassy_executor::task]
async fn uart_sender(mut uart: UartTx<'static, USART1, DMA1_CH4>) {
    let mut uart_buffer = [0; 1024];

    loop {
        let state = CONTROLLER_STATUS.wait().await;
        let msg = common::encode_embedded_message(
            &EmbeddedMessage::State(EmbeddedState {
                loop_time_us: state.loop_time_us,
                torque: state.torque,
                speed: state.speed,
                angle: state.angle,
                target_reached: state.target_reached,
            }),
            &mut uart_buffer,
        )
        .unwrap();

        uart.write(msg).await.unwrap();
    }
}

#[embassy_executor::task]
async fn uart_parser(uart: UartRx<'static, USART1, DMA1_CH5>) {
    let mut uart_buffer = [0; 1024];

    let mut rx = uart.into_ring_buffered(&mut uart_buffer);
    rx.start().unwrap();

    let mut rx_buff = [0; 1024];
    let mut valid_data_len = 0;

    loop {
        let Ok(len) = rx.read(&mut rx_buff[valid_data_len..]).await else {
            rx.start().unwrap();
            continue;
        };
        valid_data_len += len;

        let mut processed_up_to = 0;

        for msg in rx_buff[..valid_data_len].split_inclusive_mut(|val| *val == 0) {
            if msg.ends_with(&[0]) {
                // Process a complete message
                if let Ok(decoded_msg) = common::decode_host_message(msg) {
                    info!("Received data {}", Debug2Format(&decoded_msg));

                    CONTROLLER_CTL_QUEUE.send(decoded_msg).await;
                }
                processed_up_to += msg.len();
            } else {
                // Found an incomplete message, stop processing further
                break;
            }
        }

        // Check if there's unprocessed data left (either incomplete or not yet processed)
        if processed_up_to < valid_data_len {
            // Move unprocessed data to the beginning of the buffer
            let unprocessed_len = valid_data_len - processed_up_to;
            rx_buff.copy_within(processed_up_to..valid_data_len, 0);
            valid_data_len = unprocessed_len; // Update valid data length to reflect the remaining unprocessed data
        } else {
            // All data processed, reset for next read
            valid_data_len = 0;
        }

        if valid_data_len > rx_buff.len() / 2 {
            error!("Leftover from reading is bigger than half buffer size, dropping data to avoid stalling.");
            valid_data_len = 0;
        }
    }
}

#[embassy_executor::task]
async fn led_blinker(mut led: Output<'static, AnyPin>, delay: Duration) {
    let mut ticker = Ticker::every(delay);
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        ticker.next().await
    }
}

fn gd32_frequency_boost() {
    embassy_stm32::pac::RCC.cfgr().modify(|w| {
        w.set_sw(embassy_stm32::pac::rcc::vals::Sw::HSE);
    });
    loop {
        if embassy_stm32::pac::RCC.cfgr().read().sws() == embassy_stm32::pac::rcc::vals::Sw::HSE {
            break;
        }
    }

    embassy_stm32::pac::RCC.cr().modify(|w| {
        w.set_pllon(false);
    });

    embassy_stm32::pac::RCC.cfgr().modify(|w| {
        w.set_pllxtpre(embassy_stm32::pac::rcc::vals::Pllxtpre::DIV2);
        w.set_pllmul(embassy_stm32::pac::rcc::vals::Pllmul::MUL12);
        w.0 |= 1 << 27;
    });

    embassy_stm32::pac::RCC.cr().modify(|w| {
        w.set_pllon(true);
    });

    loop {
        if embassy_stm32::pac::RCC.cr().read().pllrdy() {
            break;
        }
    }

    embassy_stm32::pac::RCC.cfgr().modify(|w| {
        w.set_sw(embassy_stm32::pac::rcc::vals::Sw::PLL1_P);
    });
    loop {
        if embassy_stm32::pac::RCC.cfgr().read().sws() == embassy_stm32::pac::rcc::vals::Sw::PLL1_P
        {
            break;
        }
    }

    embassy_stm32::pac::SPI1.cr1().modify(|w| {
        w.set_br(embassy_stm32::pac::spi::vals::Br::DIV16);
    });

    embassy_stm32::pac::TIM2.psc().modify(|w| {
        w.set_psc(107);
    });

    embassy_stm32::pac::USART1.brr().write(|w| {
        w.set_brr((108000000 / 115200) as u16);
    })
}
