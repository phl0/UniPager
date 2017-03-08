use std::process::{Command, Stdio};
use std::io::Write;
use std::thread::sleep;
use std::time::Duration;
// use raspi::{Gpio, Pin, Direction, Model};
use serial::{self, SerialPort};

use pocsag::Generator;
use config::Config;
use transmitter::Transmitter;

const BAUD_RATE: usize = 1200;
const SAMPLE_RATE: usize = 48000;
const SAMPLES_PER_BIT: usize = SAMPLE_RATE/BAUD_RATE;

pub struct AudioTransmitter {
    device: String,
    ptt_type: String,
    // ptt_pin: Pin,
    ptt_port: String,
    inverted: bool,
    level: u8,
    tx_delay: usize,
    serial: Box<serial::SerialPort>
}

impl AudioTransmitter {
    pub fn new(config: &Config) -> AudioTransmitter {
        info!("Initializing audio transmitter...");
        // info!("Detected {}", Model::get());
        let ptt_type = &config.audio.ptt_type;
        info!("PTT type: {:?}", ptt_type);

        let ptt_port = &config.audio.ptt_port;
        info!("Initializing serial port {:?}", ptt_port);
        let mut serial = serial::open(&config.audio.ptt_port)
            .expect("Unable to open serial port");

//        serial.configure(&serial::PortSettings {
//            baud_rate: serial::BaudRate::Baud38400,
//            char_size: serial::CharSize::Bits8,
//            parity: serial::Parity::ParityNone,
//            stop_bits: serial::StopBits::Stop1,
//            flow_control: serial::FlowControl::FlowNone
//        }).expect("Unable to configure serial port");

        // info!("Set DTR false");
        // serial.set_dtr(false)
            // .expect("Error");

        // let gpio = Gpio::new().expect("Failed to map GPIO");

        let device = match &*config.audio.device {
            "" => String::from("default"),
            other => other.to_owned()
        };

        let mut transmitter = AudioTransmitter {
            device: device,
            ptt_type: config.audio.ptt_type.to_owned(),
            //ptt_pin: gpio.pin(config.audio.ptt_pin, Direction::Output),
            ptt_port: config.audio.ptt_port.to_owned(),
            inverted: config.audio.inverted,
            level: config.audio.level,
            tx_delay: config.audio.tx_delay,
            serial: Box::new(serial)
        };

        if transmitter.level > 127 {
            transmitter.level = 127;
        }

       //transmitter.ptt_pin.set_low();
       transmitter.serial.set_dtr(true)
           .expect("Error setting PTT pin");

       transmitter
    }
}

impl Transmitter for AudioTransmitter {
    fn send(&mut self, gen: Generator) {
        //self.ptt_pin.set_high();
        self.serial.set_dtr(false)
           .expect("Error setting PTT pin");

        sleep(Duration::from_millis(self.tx_delay as u64));

        let mut buffer: Vec<u8> = Vec::with_capacity(SAMPLE_RATE);
        let low_level = 127 - self.level;
        let high_level = 128 + self.level;

        for word in gen {
            for i in 0..32 {
                let bit = (word & (1 << (31 - i))) != 0;
                if (!self.inverted && bit) || (self.inverted && !bit) {
                    buffer.extend_from_slice(&[low_level; SAMPLES_PER_BIT]);
                }
                else {
                    buffer.extend_from_slice(&[high_level; SAMPLES_PER_BIT]);
                }
            }
        }

        let mut child = Command::new("aplay")
            .stdin(Stdio::piped())
            .args(&["-t", "raw", "-N", "-f", "U8", "-c", "1"])
            .args(&["-r", &*format!("{}", SAMPLE_RATE)])
            .args(&["-D", &*self.device])
            .spawn()
            .expect("Failed to start aplay");

        child.stdin.as_mut()
            .expect("Failed to get aplay stdin")
            .write_all(buffer.as_slice())
            .expect("Failed to write to aplay stdin");

        child.wait().expect("Failed to wait for aplay");

        //self.ptt_pin.set_low();
        self.serial.set_dtr(true)
           .expect("Error setting PTT pin");
    }
}
