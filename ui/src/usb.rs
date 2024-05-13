use std::thread::JoinHandle;

use rusb::{Context, Device, HotplugBuilder, UsbContext};
use slint::ComponentHandle;
use tokio_util::sync::CancellationToken;

use super::get_com_ports;
use super::{Properties, RsFocUi};

pub struct HotplugNotify {
    handle: slint::Weak<RsFocUi>,
}

impl HotplugNotify {
    pub fn new(handle: slint::Weak<RsFocUi>) -> Self {
        Self { handle }
    }
}

impl<T: UsbContext> rusb::Hotplug<T> for HotplugNotify {
    fn device_arrived(&mut self, _device: Device<T>) {
        let _ = self.handle.upgrade_in_event_loop(|ui| {
            ui.global::<Properties>().set_com_ports(get_com_ports());
        });
    }

    fn device_left(&mut self, _device: Device<T>) {
        let _ = self.handle.upgrade_in_event_loop(|ui| {
            ui.global::<Properties>().set_com_ports(get_com_ports());
        });
    }
}

pub struct UsbWorker {
    cancel: tokio_util::sync::CancellationToken,
    worker_thread: std::thread::JoinHandle<rusb::Result<()>>,
}

impl UsbWorker {
    pub fn new(cargo_ui: &RsFocUi) -> Self {
        let cancel = CancellationToken::new();

        let worker_thread = std::thread::spawn({
            let handle_weak = cargo_ui.as_weak();
            let cancel = cancel.clone();

            move || {
                tokio::runtime::Runtime::new()
                    .unwrap()
                    .block_on(usb_worker_loop(cancel, handle_weak))
            }
        });
        Self {
            cancel,
            worker_thread,
        }
    }

    pub fn stop(self) -> std::thread::Result<rusb::Result<()>> {
        self.cancel.cancel();
        self.worker_thread.join()
    }
}

async fn usb_worker_loop(
    cancel: CancellationToken,
    handle: slint::Weak<RsFocUi>,
) -> rusb::Result<()> {
    let context = Context::new()?;

    let reg = HotplugBuilder::new()
        .enumerate(true)
        .register(&context, Box::new(HotplugNotify::new(handle)))?;

    let loop_thread: JoinHandle<Result<(), rusb::Error>> = std::thread::spawn({
        let context = context.clone();
        let cancel = cancel.clone();
        move || loop {
            context.handle_events(None)?;
            if cancel.is_cancelled() {
                return Ok(());
            }
        }
    });

    cancel.cancelled().await;
    context.interrupt_handle_events();
    context.unregister_callback(reg);
    loop_thread.join().expect("Loop thread panicked")?;

    Ok(())
}
