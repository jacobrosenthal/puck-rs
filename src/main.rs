//! http://www.espruino.com/Puck.js
//! http://www.espruino.com/img/PUCKJS_.jpg
//! https://github.com/espruino/Espruino/blob/2e101fdb78b99b883ad627a2908e797e841494d9/libs/puckjs/jswrap_puck.c
//!
//! p0.00 btn1
//! p0.01 d1
//! p0.02 d2
//! p0.03 ledb
//! p0.04 ledg
//! p0.05 ledr
//! p0.06 temp scl
//! p0.07 temp sda
//! p0.08 temp vdd
//! p0.09 nfc
//! p0.10 nfc
//!
//! cargo run --release

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use nrf_softdevice_defmt_rtt as _; // global logger
use panic_probe as _; // print out panic messages
mod ble;

use ble::{bluetooth_task, softdevice_config, softdevice_task};
use defmt::{info, unwrap};
use embassy::executor::Spawner;
use embassy_nrf::gpio::{self, Pin};
use embassy_nrf::gpiote::{self, Channel};
use embassy_nrf::saadc::{self, Saadc};
use embassy_nrf::{interrupt, Peripherals};
use nrf_softdevice::Softdevice;
use nrf_softdevice_s132::{sd_power_dcdc_mode_set, NRF_POWER_DCDC_MODES_NRF_POWER_DCDC_ENABLE};

#[embassy::main(config = "embassy_config()")]
async fn main(spawner: Spawner, dp: Peripherals) {
    // well use these logging macros instead of println to tunnel our logs via the debug chip
    info!("Hello World!");

    // some bluetooth under the covers stuff we need to start up
    let config = softdevice_config();
    let sd = Softdevice::enable(&config);

    // save battery
    unsafe {
        sd_power_dcdc_mode_set(NRF_POWER_DCDC_MODES_NRF_POWER_DCDC_ENABLE as u8);
    }

    // button presses will be delivered on HiToLo or when you release the button
    let button1 = gpiote::InputChannel::new(
        // degrade just a typesystem hack to forget which pin it is so we can
        // call it Anypin and make our function calls more generic
        dp.GPIOTE_CH1.degrade(),
        gpio::Input::new(dp.P0_00.degrade(), gpio::Pull::Down),
        gpiote::InputChannelPolarity::HiToLo,
    );

    let blue = gpio::Output::new(
        dp.P0_03.degrade(),
        gpio::Level::Low,
        gpio::OutputDrive::Standard,
    );

    let green = gpio::Output::new(
        dp.P0_04.degrade(),
        gpio::Level::Low,
        gpio::OutputDrive::Standard,
    );

    let mut config = saadc::Config::default();
    // must change battery calculation if resolution changes
    config.resolution = saadc::Resolution::_14BIT;
    let irq = interrupt::take!(SAADC);
    let channel_config = saadc::ChannelConfig::single_ended(saadc::VddInput);
    let saadc = Saadc::new(dp.SAADC, irq, config, [channel_config]);

    // tell the executor to start each of our tasks
    unwrap!(spawner.spawn(softdevice_task(sd)));
    // note this unwrap! macro is just like .unwrap() you're used to, but for
    // various reasons has less size for microcontrollers
    unwrap!(spawner.spawn(bluetooth_task(sd, button1, blue, green, saadc)));
}

// 0 is Highest. Lower prio number can preempt higher prio number
// Softdevice has reserved priorities 0, 1 and 3
pub fn embassy_config() -> embassy_nrf::config::Config {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    config.time_interrupt_priority = interrupt::Priority::P2;
    // if we see button misses lower this
    config.gpiote_interrupt_priority = interrupt::Priority::P7;
    config
}

// just a bookkeeping function for our logging library
// WARNING may overflow and wrap-around in long lived apps
defmt::timestamp! {"{=usize}", {
    use core::sync::atomic::{AtomicUsize, Ordering};
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
}
}
