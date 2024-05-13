use common::{ControllerCalibrateEncoderStatus, ControllerMode};
use fixed::traits::ToFixed;
use plotters::prelude::*;
use ringbuffer::RingBuffer;
use slint::{ModelRc, SharedPixelBuffer, SharedString, VecModel};
use std::rc::Rc;
use uart::{UartWorker, UartWorkerCommand};
use usb::UsbWorker;

slint::include_modules!();

mod uart;
mod usb;

fn get_com_ports() -> ModelRc<SharedString> {
    Rc::new(VecModel::from(
        uart::get_com_ports()
            .unwrap()
            .into_iter()
            .map(|port| port.into())
            .collect::<Vec<_>>(),
    ))
    .into()
}

fn main() -> Result<(), slint::PlatformError> {
    let ui = RsFocUi::new()?;

    let uart_worker = UartWorker::new(&ui);
    let usb_worker = UsbWorker::new(&ui);

    ui.global::<Properties>().set_com_ports(get_com_ports());

    ui.global::<Properties>().on_connect({
        let uart_handle = uart_worker.channel.clone();
        move |port| {
            uart_handle
                .send(UartWorkerCommand::OpenPort(port.into()))
                .unwrap();
        }
    });

    ui.global::<Properties>().on_calibrate_encoder({
        let uart_handle = uart_worker.channel.clone();
        move || {
            uart_handle
                .send(UartWorkerCommand::SetMode(
                    ControllerMode::CalibrateEncoder(ControllerCalibrateEncoderStatus::Init),
                ))
                .unwrap();
        }
    });

    ui.global::<Properties>().on_update_torque_str({
        let ui = ui.as_weak();
        move |val| {
            ui.upgrade()
                .unwrap()
                .global::<Properties>()
                .set_torque_str(format!("{:0.3}", val).into());
        }
    });

    ui.global::<Properties>().on_update_speed_str({
        let ui = ui.as_weak();
        move |val| {
            ui.upgrade()
                .unwrap()
                .global::<Properties>()
                .set_speed_str(format!("{:0.3}", val).into())
        }
    });

    ui.global::<Properties>().on_update_angle_str({
        let ui = ui.as_weak();
        move |val| {
            ui.upgrade()
                .unwrap()
                .global::<Properties>()
                .set_angle_str(format!("{:0.3}", val).into());
        }
    });

    ui.global::<Properties>().on_update({
        let uart_handle = uart_worker.channel.clone();
        let ui = ui.as_weak();
        move || {
            let ui = ui.upgrade().unwrap();
            let properties = ui.global::<Properties>();

            let mode = match (
                properties.get_armed(),
                properties.get_selected_mode().as_str(),
            ) {
                (false, _) => ControllerMode::Idle,
                (_, "Torque control") => ControllerMode::Run(common::ControllerRunParams::Torque {
                    target: properties.get_torque_target().to_fixed(),
                }),
                (_, "Speed control") => {
                    ControllerMode::Run(common::ControllerRunParams::Velocity {
                        target: (properties.get_speed_target() * core::f32::consts::TAU / 60f32)
                            .to_fixed(),
                        torque_limit: properties.get_torque_target().to_fixed(),
                    })
                }
                (_, "Angle control") => ControllerMode::Run(common::ControllerRunParams::Angle {
                    target: (properties.get_angle_target() * core::f32::consts::TAU / 360f32)
                        .to_fixed(),
                    speed_limit: (properties.get_speed_target() * core::f32::consts::TAU / 60f32)
                        .to_fixed(),
                    torque_limit: properties.get_torque_target().to_fixed(),
                }),
                (_, _) => unreachable!(),
            };

            uart_handle.send(UartWorkerCommand::SetMode(mode)).unwrap();
        }
    });

    ui.global::<Properties>()
        .on_generate_encoder_calibration_chart({
            let encoder_calibration = uart_worker.encoder_calibration.clone();
            move |width, height| {
                println!("Offsets size: {}:{}", width, height);
                let offsets = encoder_calibration.borrow();
                let mut pixel_buffer = SharedPixelBuffer::new(width as u32, height as u32);
                let size = (pixel_buffer.width(), pixel_buffer.height());

                let backend = BitMapBackend::with_buffer(pixel_buffer.make_mut_bytes(), size);

                let root = backend.into_drawing_area();

                root.fill(&WHITE).expect("error filling drawing area");

                let min: f64 = offsets
                    .iter()
                    .min()
                    .unwrap_or(&0.to_fixed())
                    .min(&0.to_fixed())
                    .to_num();
                let max: f64 = offsets
                    .iter()
                    .max()
                    .unwrap_or(&0.to_fixed())
                    .max(&0.to_fixed())
                    .to_num();

                let mut chart = ChartBuilder::on(&root)
                    .build_cartesian_2d(0.0..1.0, min..max)
                    .expect("error building coordinate system");

                chart
                    .draw_series(LineSeries::new(
                        offsets.iter().enumerate().map(|(idx, val)| {
                            (idx as f64 / (offsets.len() - 1) as f64, val.to_num::<f64>())
                        }),
                        &RED,
                    ))
                    .expect("error drawing series");

                root.present().expect("error presenting");

                drop(chart);
                drop(root);

                slint::Image::from_rgb8(pixel_buffer)
            }
        });

    ui.global::<Properties>().on_generate_traces_chart({
        let state_history = uart_worker.state_history.clone();
        move |width, height| {
            println!("Traces size: {}:{}", width, height);
            let history = state_history.lock().unwrap();

            let mut pixel_buffer = SharedPixelBuffer::new(width as u32, height as u32);
            let size = (pixel_buffer.width(), pixel_buffer.height());

            let backend = BitMapBackend::with_buffer(pixel_buffer.make_mut_bytes(), size);

            let root = backend.into_drawing_area();

            root.fill(&WHITE).expect("error filling drawing area");

            let min: f64 = history
                .iter()
                .map(|value| {
                    [value.angle, value.speed.to_fixed(), value.torque.to_fixed()]
                        .into_iter()
                        .min()
                        .unwrap()
                })
                .min()
                .unwrap_or_default()
                .min((-1).to_fixed())
                .to_num();

            let max = history
                .iter()
                .map(|value| {
                    [value.angle, value.speed.to_fixed(), value.torque.to_fixed()]
                        .into_iter()
                        .max()
                        .unwrap()
                })
                .max()
                .unwrap_or_default()
                .max((1).to_fixed())
                .to_num();

            let mut chart = ChartBuilder::on(&root)
                // .margin(5)
                .set_all_label_area_size(50)
                // .caption("Sine and Cosine", ("sans-serif", 40))
                .build_cartesian_2d(0.0..1.0, min..max)
                .expect("error building coordinate system");

            chart
                .configure_mesh()
                .x_labels(20)
                .y_labels(10)
                // .disable_mesh()
                .x_label_formatter(&|v| format!("{:.1}", v))
                .y_label_formatter(&|v| format!("{:.1}", v))
                .draw()
                .unwrap();

            chart
                .draw_series(LineSeries::new(
                    history.iter().enumerate().map(|(idx, val)| {
                        (
                            idx as f64 / (history.len() - 1) as f64,
                            val.torque.to_num::<f64>(),
                        )
                    }),
                    RED,
                ))
                .expect("error drawing series")
                .label("Torque")
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

            chart
                .draw_series(LineSeries::new(
                    history.iter().enumerate().map(|(idx, val)| {
                        (
                            idx as f64 / (history.len() - 1) as f64,
                            val.speed.to_num::<f64>(),
                        )
                    }),
                    &GREEN,
                ))
                .expect("error drawing series");

            chart
                .draw_series(LineSeries::new(
                    history.iter().enumerate().map(|(idx, val)| {
                        (
                            idx as f64 / (history.len() - 1) as f64,
                            val.angle.to_num::<f64>(),
                        )
                    }),
                    &BLUE,
                ))
                .expect("error drawing series");

            drop(history);

            chart.configure_series_labels().border_style(BLACK).draw().unwrap();

            root.present().expect("error presenting");

            drop(chart);
            drop(root);

            slint::Image::from_rgb8(pixel_buffer)
        }
    });

    ui.run().expect("UI failed");
    uart_worker
        .stop()
        .expect("UartWorker panicked")
        .expect("UartWorker failed");
    usb_worker
        .stop()
        .expect("UsbWorker panicked")
        .expect("UsbWorker failed");

    Ok(())
}
