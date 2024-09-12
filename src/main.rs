#![no_std]
#![no_main]

mod fmt;

use core::convert::Infallible;
use core::str::FromStr;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::usb_otg::Out;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};
use embedded_io::ErrorType;
use fmt::{info, todo, unwrap};
use heapless::String;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embedded_cli::cli::CliBuilder;
use embedded_cli::cli::CliHandle;
use embedded_cli::{command, Command};
use ufmt::{uwrite, uwriteln};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[derive(Debug, Command)]
enum BaseCommand<'a> {
    /// Control LEDs
    Led {
        /// LED id
        #[arg(long)]
        id: u8,
        #[command(subcommand)]
        command: LedCommand,
    },
    Pin {
        #[arg(short = 'p', long)]
        pin_name: &'a str,
        #[command(subcommand)]
        command: PinCommand<'a>,
    },

    /// Show some status
    Status,
}

#[derive(Debug, Command)]
enum PinCommand<'a> {
    /// Get pin's num
    Get,
    /// Set LED value
    Set {
        /// used to config input or output
        #[arg(short = 'd', long)]
        dir: Option<&'a str>,
        /// used to config low or high
        #[arg(short = 'm', long)]
        level: Option<&'a str>,
        /// used to config speed
        #[arg(long)]
        speed: Option<&'a str>,
        #[arg(long)]
        pull: Option<&'a str>,
        #[arg(long)]
        outype: Option<&'a str>,
    },
}

#[derive(Debug, Command)]
enum LedCommand {
    /// Get current LED value
    Get,
    /// Set LED value
    Set {
        /// LED brightness
        value: u8,
    },
    /// Toggle the led
    Toggle,
}

/// Wrapper around usart so we can impl embedded_io::Write
/// which is required for cli
struct Writer<'a>(usart::UartTx<'a, peripherals::USART1>);

impl<'a> ErrorType for Writer<'a> {
    type Error = Infallible;
}

impl<'a> embedded_io::Write for Writer<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        unwrap!(self.0.blocking_write(buf));
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        unwrap!(self.0.blocking_flush());
        Ok(())
    }
}

struct AppState<'a> {
    led: Output<'a, peripherals::PC13>,
    led_brightness: [u8; 4],
    num_commands: usize,
}

fn on_led(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    state: &mut AppState,
    id: u8,
    command: LedCommand,
) -> Result<(), Infallible> {
    state.num_commands += 1;

    if id as usize > state.led_brightness.len() {
        uwriteln!(cli.writer(), "LED cmd recv!")?;
    } else {
        match command {
            LedCommand::Get => {}
            LedCommand::Set { value } => {
                state.led_brightness[id as usize] = value;
                uwrite!(cli.writer(), "recv Set value = {}", value)?;
            }
            LedCommand::Toggle => {
                state.led.toggle();
            }
        }
    }

    Ok(())
}

fn get_pin_num(pin_name: &str) -> Option<u8> {
    if pin_name.len() > 4 || pin_name.len() < 2 {
        return None;
    }

    let mut pin_name: String<8> = String::from_str(pin_name).unwrap();
    pin_name.make_ascii_uppercase();

    let mut pin_name = pin_name.chars();
    let mut port_num = 0u8;
    let mut pin_num = 0u8;
    if let Some(port) = pin_name.next() {
        if !('A'..='Z').contains(&port) {
            return None;
        }
        port_num = port as u8 - b'A';
    }
    if let Ok(num) = pin_name.as_str().parse::<u8>() {
        if num > 15 {
            return None;
        }
        pin_num = num;
    }

    Some(port_num * 16 + pin_num)
}

fn on_pin(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    state: &mut AppState,
    pin_name: &str,
    command: PinCommand,
) -> Result<(), Infallible> {
    state.num_commands += 1;
    match command {
        PinCommand::Get => {
            if let Some(num) = get_pin_num(pin_name) {
                uwrite!(cli.writer(), "get pin number = {}", num)?;
            } else {
                uwrite!(cli.writer(), "pin name should be like 'A9', 'B15'...")?;
            }
        }
        PinCommand::Set {
            dir,
            level,
            speed,
            pull,
            outype,
        } => {
            let mut pin = 0;
            if let Some(num) = get_pin_num(pin_name) {
                pin = num;
                uwrite!(cli.writer(), "get pin number = {}", num)?;
            } else {
                uwrite!(cli.writer(), "pin name should be like 'A9', 'B15'...")?;
                return Ok(());
            }
            let pin = unsafe { AnyPin::steal(pin) };

            let mut is_output = false;
            if let Some(dir) = dir {
                let mut dir: String<8> = String::from_str(dir).unwrap();
                dir.make_ascii_lowercase();
                let dir = dir.as_str();
                if dir == "input" {
                    is_output = false;
                } else if dir == "output" {
                    is_output = true;
                } else {
                    uwrite!(cli.writer(), "dir should be 'input' or 'output'...")?;
                    return Ok(());
                }
            }
            let mut spd = Speed::VeryHigh;
            if let Some(s) = speed {
                let mut s: String<8> = String::from_str(s).unwrap();
                s.make_ascii_lowercase();
                let s = s.as_str();
                if s == "veryhigh" {
                    spd = Speed::VeryHigh;
                } else if s == "high" {
                    spd = Speed::High;
                } else if s == "medium" {
                    spd = Speed::Medium;
                } else if s == "low" {
                    spd = Speed::Low;
                }
            }

            let mut pt = Pull::Up;
            if let Some(p) = pull {
                let mut p: String<8> = String::from_str(p).unwrap();
                p.make_ascii_lowercase();
                let p = p.as_str();
                if p == "up" {
                    pt = Pull::Up;
                } else if p == "down" {
                    pt = Pull::Down;
                }
            }

            let mut _out_type = OutputType::PushPull;
            if let Some(o) = outype {
                let mut o: String<12> = String::from_str(o).unwrap();
                o.make_ascii_lowercase();
                let o = o.as_str();
                if o == "opendrain" {
                    _out_type = OutputType::OpenDrain;
                } else if o == "pushpull" {
                    _out_type = OutputType::PushPull;
                }
            }

            let mut lvl = Level::High;
            if let Some(l) = level {
                let mut l: String<8> = String::from_str(l).unwrap();
                l.make_ascii_lowercase();
                let l = l.as_str();
                if l == "low" {
                    lvl = Level::Low;
                } else if l == "high" {
                    lvl = Level::High;
                }
            }

            if is_output {
                let _ = Output::new(pin, lvl, spd);
            } else {
                let _ = Input::new(pin, pt);
            }
        }
    }

    Ok(())
}

fn on_status(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    state: &mut AppState,
) -> Result<(), Infallible> {
    state.num_commands += 1;
    uwrite!(cli.writer(), "Total Cmds Received: {}", state.num_commands)?;
    Ok(())
}

#[embassy_executor::task]
async fn cli_task(uart: peripherals::USART1, pin1: peripherals::PA10, pin2: peripherals::PA9) {
    let config = UartConfig::default();

    let usart = Uart::new(
        uart,
        pin1,
        pin2,
        Irqs,
        embassy_stm32::dma::NoDma,
        embassy_stm32::dma::NoDma,
        config,
    )
    .unwrap();

    let (tx, mut rx) = usart.split();
    let tx = Writer(tx);

    // create static buffers for use in cli (so we're not using stack memory)
    // History buffer is 1 byte longer so max command fits in it (it requires extra byte at end)
    // SAFETY: buffers are passed to cli and are used by cli only
    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 128] = [0; 128];
        static mut HISTORY_BUFFER: [u8; 128] = [0; 128];
        (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
    };
    let mut cli = unwrap!(CliBuilder::default()
        .writer(tx)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build()
        .ok());

    // Create global state, that will be used for entire application

    let led_pin = Output::new(
        unsafe { peripherals::PC13::steal() },
        Level::High,
        Speed::VeryHigh,
    );

    let mut state = AppState {
        led: led_pin,
        led_brightness: [0; 4],
        num_commands: 0,
    };

    let _ = cli.write(|writer| {
        // storing big text in progmem
        // for small text it's usually better to use normal &str literals
        uwrite!(
            writer,
            "Cli is running.
Type \"help\" for a list of commands.
Use backspace and tab to remove chars and autocomplete.
Use up and down for history navigation.
Use left and right to move inside input."
        )
    });

    let mut byte = [0u8; 1];
    loop {
        unwrap!(rx.blocking_read(&mut byte));
        // Process incoming byte
        // Command type is specified for autocompletion and help
        // Processor accepts closure where we can process parsed command
        // we can use different command and processor with each call
        let _ = cli.process_byte::<BaseCommand, _>(
            byte[0],
            &mut BaseCommand::processor(|cli, command| match command {
                BaseCommand::Led { id, command } => on_led(cli, &mut state, id, command),
                BaseCommand::Status => on_status(cli, &mut state),
                BaseCommand::Pin { pin_name, command } => {
                    on_pin(cli, &mut state, pin_name, command)
                }
            }),
        );
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    unwrap!(spawner.spawn(cli_task(p.USART1, p.PA10, p.PA9)));

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
