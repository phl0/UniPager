extern crate serial;
extern crate raspi;
extern crate ws;
extern crate tiny_http;
extern crate serde;
extern crate hidapi;
#[macro_use]
extern crate serde_derive;
#[macro_use]
extern crate serde_json;
#[macro_use]
extern crate log;
#[macro_use]
extern crate lazy_static;
extern crate crossbeam_channel;

#[macro_use]
mod status;
mod config;
mod logging;
mod connection;
mod transmitter;
mod pocsag;
mod frontend;

use std::thread;
use std::time;
use std::fs::File;
use std::io::Read;

use config::Config;
use connection::Connection;
use frontend::{Request, Response};
use pocsag::Scheduler;

fn print_version() {
    println!("UniPager {}", env!("CARGO_PKG_VERSION"));
    println!("Copyright (c) 2017 RWTH Amateurfunkgruppe\n");
    println!("This program comes with ABSOLUTELY NO WARRANTY.");
    println!("This is free software, and you are welcome to redistribute");
    println!("and modify it under the conditions of the GNU GPL v3 or later.");
    println!("<https://www.gnu.org/licenses/gpl-3.0.txt>\n");
}

fn main() {
    print_version();

    let pass = File::open("password")
        .and_then(|mut f| {
            let mut s = String::new();
            f.read_to_string(&mut s)?;
            Ok(s)
        })
        .map(|s| s.trim().to_owned())
        .ok();

    let (responder, requests) = frontend::run(pass.as_ref().map(|x| &**x));


    logging::init(responder.clone());
    status::subscribe(responder.clone());

    let mut config = Config::load();
    let scheduler = Scheduler::new(&config);

    thread::spawn(timeslot_updater);

    let mut restart = true;
    let mut test = false;

    while restart {
        let (stop_conn, conn_thread) =
            Connection::start(config.clone(), scheduler.clone());
        let scheduler_thread =
            Scheduler::start(config.clone(), scheduler.clone());
        loop {
            match requests.recv().unwrap() {
                Request::SetConfig(new_config) => {
                    config = new_config;
                    config.save();
                    responder.send(Response::Config(config.clone()));
                    info!("Config updated. Initiating restart.");

                    restart = true;
                    stop_conn.send(()).ok();
                    scheduler.stop();
                    break;
                }
                Request::DefaultConfig => {
                    config = Config::default();
                    config.save();
                    responder.send(Response::Config(config.clone()));
                    info!("Config set to default. Initiating restart.");

                    restart = true;
                    stop_conn.send(()).ok();
                    scheduler.stop();
                    break;
                }
                Request::SendMessage(msg) => {
                    let msg_copy = msg.clone();
                    scheduler.message(msg);
                    responder.send(Response::Message(msg_copy));
                }
                Request::GetConfig => {
                    responder.send(Response::Config(config.clone()));
                }
                Request::GetVersion => {
                    let version = env!("CARGO_PKG_VERSION").to_string();
                    responder.send(Response::Version(version));
                }
                Request::GetStatus => {
                    responder.send(Response::Status(status::get()));
                }
                Request::Shutdown => {
                    info!("Initiating shutdown.");
                    restart = false;
                    stop_conn.send(()).ok();
                    scheduler.stop();
                    break;
                }
                Request::Restart => {
                    info!("Initiating restart.");
                    restart = true;
                    stop_conn.send(()).ok();
                    scheduler.stop();
                    break;
                }
                Request::Test => {
                    info!("Initiating test procedure...");
                    restart = true;
                    test = true;
                    stop_conn.send(()).ok();
                    scheduler.stop();
                    break;
                }
                Request::Authenticate(_) => {
                    break;
                }
            }
        }

        info!("Waiting for the connection to terminate...");
        conn_thread.join().ok();

        info!("Waiting for the scheduler to terminate...");
        scheduler_thread.join().ok();
        info!("Scheduler stopped.");

        if test {
            info!("Starting test transmission.");
            let thread = Scheduler::test(config.clone(), scheduler.clone());
            thread.join().ok();
            info!("Test transmission completed. Restarting...");
            test = false;
        }
    }

    info!("Terminating... 73!");
    thread::sleep(time::Duration::from_millis(1000));
}

pub fn timeslot_updater() {
    loop {
        let timeslot = pocsag::TimeSlot::current();
        status!(timeslot: timeslot);
        thread::sleep(timeslot.next().duration_until());
    }
}
