use raspi::{Direction, Gpio, Model, Pin};
use serial::{self, SerialPort};
use std::{thread, time};
use std::io::{Write};

use config::Config;
use transmitter::Transmitter;

pub struct C9000Transmitter {
    reset_pin: Box<dyn Pin>,
    ptt_pin: Box<dyn Pin>,
    send_pin: Box<dyn Pin>,
    status_led_pin: Box<dyn Pin>,
    connected_led_pin: Box<dyn Pin>,
    serial: Box<dyn serial::SerialPort>
}

impl C9000Transmitter {
    pub fn new(config: &Config) -> C9000Transmitter {
        info!("Initializing C9000 transmitter...");

        if config.c9000.dummy_enabled {
            info!("Setting C9000 PA dummy output power");

            if let Ok(mut port) = serial::open(&*config.c9000.dummy_port) {
                if port
                       .configure(&serial::PortSettings {
                           baud_rate: serial::BaudRate::Baud38400,
                           char_size: serial::CharSize::Bits8,
                           parity: serial::Parity::ParityNone,
                           stop_bits: serial::StopBits::Stop1,
                           flow_control: serial::FlowControl::FlowNone
                       }).is_err() {
                    error!("Unable to configure serial port {}", config.c9000.dummy_port);
                } else {
                    if port.write_all(&[config.c9000.dummy_pa_output_level]).is_err() {
                        error!("Unable to write data to the serial port {}", config.c9000.dummy_port);
                    }
                }
            } else {
               error!("Unable to open serial port {}", config.c9000.dummy_port);
            }

        }

        let model = Model::get();
        info!("Detected {}", model);

        let mut serial = serial::open(model.serial_port()).expect(
            "Unable to open serial port"
        );

        serial
            .configure(&serial::PortSettings {
                baud_rate: serial::BaudRate::Baud38400,
                char_size: serial::CharSize::Bits8,
                parity: serial::Parity::ParityNone,
                stop_bits: serial::StopBits::Stop1,
                flow_control: serial::FlowControl::FlowNone
            })
            .expect("Unable to configure serial port");

        let gpio = Gpio::new().expect("Failed to map GPIO");

        let transmitter = C9000Transmitter {
            reset_pin: gpio.pin(0, Direction::Output),
            ptt_pin: gpio.pin(2, Direction::Output),
            send_pin: gpio.pin(3, Direction::Input),
            status_led_pin: gpio.pin(10, Direction::Output),
            connected_led_pin: gpio.pin(11, Direction::Output),
            serial: Box::new(serial)
        };

        transmitter.reset_pin.set_high();
        transmitter.status_led_pin.set_high();
        transmitter.connected_led_pin.set_high();

        transmitter
    }
}

impl Transmitter for C9000Transmitter {
    fn send(&mut self, gen: &mut dyn Iterator<Item = u32>) {
        self.ptt_pin.set_high();

        for (i, word) in gen.enumerate() {
            if i % 10 == 0 {
                if (*self.serial).flush().is_err() {
                    error!("Unable to flush serial port");
                }

                thread::sleep(time::Duration::from_millis(10));
                while !self.send_pin.read() {
                    thread::sleep(time::Duration::from_millis(1));
                }
            }

            let bytes = [
                ((word & 0xff000000) >> 24) as u8,
                ((word & 0x00ff0000) >> 16) as u8,
                ((word & 0x0000ff00) >> 8) as u8,
                (word & 0x000000ff) as u8,
            ];

            if (*self.serial).write_all(&bytes).is_err() {
                error!("Unable to write data to the serial port");
                self.ptt_pin.set_low();
                return;
            }
        }

        if (*self.serial).flush().is_err() {
            error!("Unable to flush serial port");
        }

        self.ptt_pin.set_low();
    }
}
