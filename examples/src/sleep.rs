//! Basic Embassy example for the Raspberry Pi Pico 2
//! Showcase sleep and wake up functionality of the DFPlayer Mini
//!
//! Assumes the following connections:
//!
//! - UART0 TX/RX on GPIO 17/16
//! - Busy Pin on GPIO 18
//!
//! Assumes the following for the DFPlayer Mini:
//!
//! - Powered and ground connected
//! - TX/RX connected
//! - Busy Pin connected
//! - Speaker connected
//! - Using SD card for audio files. Place mp3 files on the SD card into a folder structure as documented in the DFPlayer Mini manual. For example:
//!     - no folders, use root directory
//!         - 0001-whatevernameX.mp3
//!         - 0002-whatevernameY.mp3
//!         - 0003-whatevernameZ.mp3

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info};
use dfplayer_async::{DfPlayer, TimeSource};
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    block::ImageDef,
    config::Config,
    gpio::{Input, Pull},
    peripherals::UART0,
    uart::{
        BufferedInterruptHandler, BufferedUart, Config as UartConfig, DataBits,
        Parity, StopBits,
    },
};
use embassy_time::{Delay, Duration, Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

bind_interrupts!(pub struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

/// Firmware entry point
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Initialize the DFPlayer Mini
    // The modules usually have a busy pin that can be used to determine if the module is currently playing audio. Low if busy, high if not busy.
    let mut busy = Input::new(p.PIN_18, Pull::None);

    // We need a UART port to communicate with the DFPlayer Mini
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 9600;
    uart_config.data_bits = DataBits::DataBits8;
    uart_config.stop_bits = StopBits::STOP1;
    uart_config.parity = Parity::ParityNone;

    // Create a buffered UART instance
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 64])[..];
    static RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 64])[..];
    let mut uart = BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_16,
        p.PIN_17,
        tx_buf,
        rx_buf,
        uart_config,
    );

    // Configure DFPlayer parameters
    let feedback_enable = true;
    let timeout_ms = 1000;
    let delay = Delay;
    let reset_duration_override = None;

    // Implement the TimeSource trait for the DFPlayer Mini
    struct MyTimeSource;
    impl TimeSource for MyTimeSource {
        type Instant = Instant;

        fn now(&self) -> Self::Instant {
            Instant::now()
        }

        fn is_elapsed(&self, since: Self::Instant, timeout_ms: u64) -> bool {
            Instant::now().duration_since(since)
                >= Duration::from_millis(timeout_ms)
        }
    }

    // Create the DFPlayer Mini instance
    let mut dfplayer = match DfPlayer::new(
        &mut uart,
        feedback_enable,
        timeout_ms,
        MyTimeSource,
        delay,
        reset_duration_override,
    )
    .await
    {
        Ok(dfplayer) => dfplayer,
        Err(e) => {
            error!("Error initializing DFPlayer Mini: {}", Debug2Format(&e));
            return;
        }
    };

    // Set the volume to a low level
    let volume = 1;
    match dfplayer.set_volume(volume).await {
        Ok(_) => info!("Volume {} set successfully", volume),
        Err(e) => error!("Failed to set volume: {}", Debug2Format(&e)),
    }

    // Play the first track for a few seconds, then stop
    info!("Playing track 1");
    match dfplayer.play(1).await {
        Ok(_) => info!("Play track 1 command sent successfully"),
        Err(e) => error!("Failed to play track: {}", Debug2Format(&e)),
    }
    Timer::after(Duration::from_secs(5)).await;
    match dfplayer.stop().await {
        Ok(_) => info!("Stop command sent successfully"),
        Err(e) => error!("Failed to stop track: {}", Debug2Format(&e)),
    }

    // Wait a moment before putting the device to sleep
    Timer::after(Duration::from_secs(1)).await;

    // Put the device to sleep
    info!("Putting DFPlayer Mini to sleep");
    match dfplayer.sleep().await {
        Ok(_) => info!("Sleep command sent successfully"),
        Err(e) => error!("Failed to put device to sleep: {}", Debug2Format(&e)),
    }

    // Wait while the device is in sleep mode
    info!("Device is now in sleep mode. Waiting for 5 seconds...");
    Timer::after(Duration::from_secs(5)).await;

    // Wake up the device
    // info!("Waking up DFPlayer Mini");
    // match dfplayer.wake_up(true, None).await {
    //     Ok(_) => info!("Wake up command sent successfully"),
    //     Err(e) => error!("Failed to wake up device: {}", Debug2Format(&e)),
    // }
    // Wake up the device
    info!("Waking up DFPlayer Mini");
    match dfplayer.reset(None).await {
        Ok(_) => info!("Wake up command sent successfully"),
        Err(e) => error!("Failed to wake up device: {}", Debug2Format(&e)),
    }

    // Wait a moment for the device to fully wake up
    Timer::after(Duration::from_secs(1)).await;

    // Play another track to verify the device is working after waking up
    info!("Playing track 2 to verify device is awake");
    match dfplayer.play(2).await {
        Ok(_) => info!("Play track 2 command sent successfully"),
        Err(e) => error!("Failed to play track: {}", Debug2Format(&e)),
    }
    Timer::after(Duration::from_secs(5)).await;
    match dfplayer.stop().await {
        Ok(_) => info!("Stop command sent successfully"),
        Err(e) => error!("Failed to stop track: {}", Debug2Format(&e)),
    }

    // Example complete
    info!("Sleep/wake up example complete");

    // Loop forever
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
