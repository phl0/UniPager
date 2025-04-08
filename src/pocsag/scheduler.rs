use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::mpsc::{RecvTimeoutError, TryRecvError};
use std::thread::{self, JoinHandle};

use config::Config;
use pocsag::{Generator, Message, MessageProvider, TestGenerator, TimeSlots};
use transmitter::{self, Transmitter};

enum Command {
    Message(Message),
    SetTimeSlots(TimeSlots),
    Stop
}

#[derive(Clone)]
pub struct Scheduler {
    tx: Sender<Command>,
    scheduler: Arc<Mutex<SchedulerCore>>
}

struct SchedulerCore {
    rx: Receiver<Command>,
    slots: TimeSlots,
    queue: VecDeque<Message>,
    stop: bool,
    budget: usize
}

impl Scheduler {
    pub fn new(_: &Config) -> Scheduler {
        let (tx, rx) = channel();

        let core = SchedulerCore {
            rx: rx,
            slots: TimeSlots::new(),
            queue: VecDeque::new(),
            stop: false,
            budget: 0
        };

        Scheduler {
            tx: tx,
            scheduler: Arc::new(Mutex::new(core))
        }
    }

    pub fn start(config: Config, scheduler: Scheduler) -> JoinHandle<()> {
        thread::spawn(move || {
            let transmitter = transmitter::from_config(&config);
            scheduler.scheduler.lock().unwrap().run(transmitter);
        })
    }

    pub fn test(config: Config, scheduler: Scheduler) -> JoinHandle<()> {
        thread::spawn(move || {
            let transmitter = transmitter::from_config(&config);
            scheduler.scheduler.lock().unwrap().test(transmitter);
        })
    }

    pub fn set_time_slots(&self, slots: TimeSlots) -> bool {
        info!("Set {:?}", slots);
        status!(timeslots: slots);
        self.tx.send(Command::SetTimeSlots(slots)).is_ok()
    }

    pub fn message(&self, msg: Message) -> bool {
        info!("Received {:?}", msg);
        status_inc!(calls_rx: 1);
        self.tx.send(Command::Message(msg)).is_ok()
    }

    pub fn stop(&self) -> bool {
        self.tx.send(Command::Stop).is_ok()
    }
}

impl SchedulerCore {
    pub fn run(&mut self, mut transmitter: Box<dyn Transmitter>) {
        info!("Scheduler started.");
        while !self.stop {
            let mut message = self.queue.pop_front();

            while message.is_none() {
                match self.rx.recv() {
                    Ok(Command::Message(msg)) => {
                        message = Some(msg);
                    }
                    Ok(Command::SetTimeSlots(slots)) => {
                        self.slots = slots;
                    }
                    Ok(Command::Stop) |
                    Err(_) => {
                        return;
                    }
                }
            }

            status!(queue: self.queue.len() + 1);

            // Calculate remaining time budget
            self.budget = self.slots.calculate_budget();

            if self.budget > 30 {
                /* transmit immediately */
            } else if let Some(next_slot) = self.slots.next_allowed() {
                let mut duration = next_slot.duration_until();

                debug!(
                    "Waiting {} seconds until {:?}...",
                    duration.as_secs(),
                    next_slot
                );

                // Process other commands while waiting for the time slot
                'waiting: while !next_slot.active() {
                    duration = next_slot.duration_until();

                    match self.rx.recv_timeout(duration) {
                        Ok(Command::Message(msg)) => {
                            self.queue.push_back(msg);
                            status!(queue: self.queue.len() + 1);
                        }
                        Ok(Command::SetTimeSlots(slots)) => {
                            self.slots = slots;
                        }
                        Ok(Command::Stop) |
                        Err(RecvTimeoutError::Disconnected) => {
                            self.queue.push_front(message.unwrap());
                            return;
                        }
                        Err(RecvTimeoutError::Timeout) => {
                            break 'waiting;
                        }
                    }
                }

                self.budget = self.slots.calculate_budget();
            } else {
                warn!("No allowed time slots! Sending anyway...");
                self.budget = usize::max_value();
            }

            status!(queue: self.queue.len(), transmitting: true);
            status_inc!(calls_tx: 1);

            debug!("Available time budget: {}", self.budget);
            transmitter.send(
                &mut Generator::new(self, message.unwrap())
            );

            status!(transmitting: false);
        }
    }

    pub fn test(&mut self, mut transmitter: Box<dyn Transmitter>) {
        status!(transmitting: true);
        transmitter.send(&mut TestGenerator::new(1125));
        status!(transmitting: false);
    }
}

impl MessageProvider for SchedulerCore {
    fn next(&mut self, count: usize) -> Option<Message> {
        debug!(
            "Remaining time budget: {}",
            self.budget as i32 - count as i32
        );

        if count + 30 > self.budget {
            return None;
        }

        loop {
            match (*self).rx.try_recv() {
                Ok(Command::Message(msg)) => {
                    self.queue.push_back(msg);
                }
                Ok(Command::SetTimeSlots(slots)) => {
                    self.slots = slots;
                }
                Ok(Command::Stop) |
                Err(TryRecvError::Disconnected) => {
                    self.stop = true;
                    return None;
                }
                Err(TryRecvError::Empty) => {
                    break;
                }
            };
        }

        let message = self.queue.pop_front();
        status!(queue: self.queue.len());

        if message.is_some() {
            status_inc!(calls_tx: 1);
        }

        message
    }
}
