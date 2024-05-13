use std::sync::{Arc, Mutex};

use common::types::RawAngle;
use common::EmbeddedState;
use ringbuffer::{AllocRingBuffer, RingBuffer};
use slint::ComponentHandle;
use tokio::io::BufReader;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
use tokio::select;
use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};
use tokio_serial::SerialPortBuilderExt;
use tokio_serial::SerialPortType;
use tokio_serial::SerialStream;
use tokio_util::sync::CancellationToken;

use super::{Properties, RsFocUi};

pub fn get_com_ports() -> Result<Vec<String>, tokio_serial::Error> {
    Ok(tokio_serial::available_ports()?
        .into_iter()
        .filter_map(|port| {
            if let SerialPortType::UsbPort(info) = port.port_type {
                if port.port_name.contains("tty") {
                    let name = format!(
                        "{}({})",
                        info.product.unwrap_or_else(|| "Unknown".to_string()),
                        port.port_name
                    );
                    return Some(name);
                }
            }

            None
        })
        .collect())
}

fn extract_port_path(s: &str) -> Option<&str> {
    if let Some(start) = s.find('(') {
        if let Some(end) = s.find(')') {
            return Some(&s[start + 1..end]);
        }
    }
    None
}

#[derive(Debug)]
pub enum UartWorkerCommand {
    OpenPort(String),
    SetMode(common::ControllerMode),
}

pub struct UartWorker {
    cancel: CancellationToken,
    worker_thread: std::thread::JoinHandle<Result<(), UartWorkerError>>,
    pub channel: UnboundedSender<UartWorkerCommand>,
    pub encoder_calibration: tokio::sync::watch::Receiver<Vec<RawAngle>>,
    pub state_history: Arc<Mutex<AllocRingBuffer<EmbeddedState>>>,
}

impl UartWorker {
    pub fn new(cargo_ui: &RsFocUi) -> Self {
        let cancel = CancellationToken::new();
        let (channel, r) = tokio::sync::mpsc::unbounded_channel();
        let (encoder_tx, encoder_calibration) = tokio::sync::watch::channel(Default::default());
        let state_history = Arc::new(Mutex::new(AllocRingBuffer::new(10000)));

        let worker_thread = std::thread::spawn({
            let handle_weak = cargo_ui.as_weak();
            let cancel = cancel.clone();
            let state_history = state_history.clone();

            move || {
                tokio::runtime::Runtime::new()?.block_on(
                    UartWorkerLoop::new(cancel, handle_weak, r, encoder_tx, state_history).run(),
                )
            }
        });
        Self {
            cancel,
            worker_thread,
            channel,
            encoder_calibration,
            state_history,
        }
    }

    pub fn stop(self) -> std::thread::Result<Result<(), UartWorkerError>> {
        self.cancel.cancel();
        self.worker_thread.join()
    }
}

use thiserror::Error;

#[derive(Debug, Error)]
pub enum UartWorkerError {
    #[error("IO error")]
    IoError(#[from] std::io::Error),
    #[error("Slint eventloop error")]
    SlintEventloopFailed(slint::EventLoopError),
    #[error("Serial error")]
    SerialBusError(#[from] tokio_serial::Error),
    #[error("Serialization error")]
    SerializationFailed(#[from] common::Error),
}

struct UartWorkerLoop {
    cancel: CancellationToken,
    ui: slint::Weak<RsFocUi>,
    rx: UnboundedReceiver<UartWorkerCommand>,
    encoder_tx: tokio::sync::watch::Sender<Vec<RawAngle>>,
    state_history: Arc<Mutex<AllocRingBuffer<EmbeddedState>>>,
    maybe_port: Option<BufReader<SerialStream>>,
    rx_data: Vec<u8>,
    state_draw_decimation: usize,
}

impl UartWorkerLoop {
    fn new(
        cancel: CancellationToken,
        ui: slint::Weak<RsFocUi>,
        rx: UnboundedReceiver<UartWorkerCommand>,
        encoder_tx: tokio::sync::watch::Sender<Vec<RawAngle>>,
        state_history: Arc<Mutex<AllocRingBuffer<EmbeddedState>>>,
    ) -> Self {
        Self {
            cancel,
            ui,
            rx,
            encoder_tx,
            state_history,
            maybe_port: None,
            rx_data: Vec::with_capacity(1000),
            state_draw_decimation: 0,
        }
    }

    fn port_failed(&mut self) -> Result<(), UartWorkerError> {
        self.maybe_port = None;
        self.update_ui(|ui| {
            ui.global::<Properties>().set_connected(false);
        })
    }

    fn handle_embedded_message(
        &mut self,
        msg: common::EmbeddedMessage,
    ) -> Result<(), UartWorkerError> {
        match msg {
            common::EmbeddedMessage::State(
                state @ common::EmbeddedState {
                    loop_time_us,
                    speed,
                    angle,
                    torque,
                    target_reached,
                },
            ) => {
                self.state_history.lock().unwrap().push(state);

                self.state_draw_decimation += 1;
                let update_graph = if self.state_draw_decimation > 50 {
                    self.state_draw_decimation = 0;
                    true
                } else {
                    false
                };

                self.update_ui(move |ui| {
                    let properties = ui.global::<Properties>();

                    properties.set_target_reached(target_reached);
                    properties.set_loop_time(format!("{:0.0}us", loop_time_us as f32).into());
                    properties.set_torque(format!("{:0.3}N", torque.to_num::<f32>()).into());
                    properties.set_speed(
                        format!(
                            "{:0.3} RPM",
                            speed.to_num::<f32>() * 60f32 / core::f32::consts::TAU
                        )
                        .into(),
                    );
                    properties.set_angle(
                        format!(
                            "{:0.3} Deg",
                            angle.to_num::<f32>() * 360f32 / core::f32::consts::TAU
                        )
                        .into(),
                    );
                    if update_graph {
                        ui.invoke_update_traces_chart();
                    }
                })
            }
            common::EmbeddedMessage::OffsetTable(offsets) => {
                println!("Offset table[{}]: {:?}", offsets.len(), offsets);

                self.encoder_tx.send_replace(offsets);
                self.update_ui(|ui| {
                    ui.invoke_update_encoder_calibration_chart();
                })
            }
        }
    }

    async fn handle_uart_command(&mut self, cmd: UartWorkerCommand) -> Result<(), UartWorkerError> {
        match cmd {
            UartWorkerCommand::OpenPort(path) => {
                self.update_ui(|ui| {
                    ui.global::<Properties>().set_connected(true);
                })?;

                if let Some(mut old_port) = self.maybe_port.take() {
                    old_port.shutdown().await?;
                }

                // As we are changing the port, clean the rx data as well.
                self.rx_data.clear();

                self.maybe_port = Some(BufReader::new(
                    tokio_serial::new(extract_port_path(&path).unwrap(), 115200)
                        .open_native_async()?,
                ));
                self.update_ui(|ui| {
                    ui.global::<Properties>().set_connected(true);
                })?;
            }
            UartWorkerCommand::SetMode(val) => {
                let msg = common::ControllerCommand::SetMode(val);
                self.send_message(msg).await?;
            }
        }

        Ok(())
    }

    fn update_ui(
        &mut self,
        func: impl FnOnce(RsFocUi) + Send + 'static,
    ) -> Result<(), UartWorkerError> {
        self.ui
            .upgrade_in_event_loop(func)
            .map_err(UartWorkerError::SlintEventloopFailed)
    }

    async fn send_message(
        &mut self,
        msg: common::ControllerCommand,
    ) -> Result<(), UartWorkerError> {
        if let Some(port) = &mut self.maybe_port {
            let data = common::encode_host_message(&msg)?;
            if port.write_all(&data).await.is_err() {
                // Assume that port has vanished.
                self.port_failed()?;
            }
        }

        Ok(())
    }

    async fn run(mut self) -> Result<(), UartWorkerError> {
        async fn conditional_reader(
            port: &mut Option<BufReader<SerialStream>>,
            buff: &mut Vec<u8>,
        ) -> Option<Result<usize, std::io::Error>> {
            match port {
                Some(port) => Some(port.read_until(0, buff).await),
                None => None,
            }
        }

        loop {
            select! {
                Some(result) = conditional_reader(&mut self.maybe_port, &mut self.rx_data) => {
                    match result {
                        Ok(0) => {
                            // EOF shouldn't happen with UART, assume that port has vanished.
                            self.port_failed()?;
                        }
                        Ok(_) => {
                            // println!("Received data {:?}", self.rx_data);
                            if let Ok(msg) = common::decode_embedded_message(&mut self.rx_data.clone()) {
                                self.handle_embedded_message(msg)?;
                            }
                            self.rx_data.clear();
                        }
                        Err(_) => {
                            // Assume that port has vanished.
                            self.port_failed()?;
                        }
                    }
                },
                maybe_command = self.rx.recv()  => {
                    if let Some(cmd) = maybe_command {
                        self.handle_uart_command(cmd).await?;
                    } else {
                        return Ok(())
                    }
                },
                _ = self.cancel.cancelled() => {
                    if let Some(mut old_port) = self.maybe_port.take() {
                        old_port.shutdown().await?;
                    }
                    return Ok(());
                }
            }
        }
    }
}
