#![no_std]
#![no_main]

mod fmt;

use core::convert::Infallible;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};
use embedded_io::ErrorType;
use fmt::{info, todo, unwrap};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embedded_cli::cli::CliBuilder;
use embedded_cli::cli::CliHandle;
use embedded_cli::Command;
use ufmt::{uwrite, uwriteln};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[derive(Debug, Command)]
enum BaseCommand {
    /// Control LEDs
    Led {
        /// LED id
        #[arg(long)]
        id: u8,
        #[command(subcommand)]
        command: LedCommand,
    },

    /// Show some status
    Status,
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
        static mut COMMAND_BUFFER: [u8; 40] = [0; 40];
        static mut HISTORY_BUFFER: [u8; 41] = [0; 41];
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
