use core::cell::RefCell;

use defmt::{info, unwrap};
use embassy::blocking_mutex::ThreadModeMutex;
use embassy::time::{Duration, Timer};
use embassy::util::{select, select3, Either, Either3};
use embassy_nrf::gpio::{self, Pin};
use embassy_nrf::gpiote::{self, Channel};
use embassy_nrf::interrupt::{self, SAADC};
use embassy_nrf::saadc::{self, Saadc};
use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};

// define a bluetooth service with one characteristic we can write and read to
#[nrf_softdevice::gatt_service(uuid = "9e7312e0-2354-11eb-9f10-fbc30a62cf38")]
struct LedService {
    #[characteristic(uuid = "9e7312e0-2354-11eb-9f10-fbc30a63cf38", read, write)]
    led: u8,
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_server]
struct Server {
    bas_service: BatteryService,
    led_service: LedService,
}

// tasks in same executor guaranteed to not be running at same time as they cant interupt eachother
static SERVER: ThreadModeMutex<RefCell<Option<Server>>> = ThreadModeMutex::new(RefCell::new(None));

#[embassy::task]
pub async fn bluetooth_task(sd: &'static Softdevice) {
    let mut dp = unsafe { <embassy_nrf::Peripherals as embassy::util::Steal>::steal() };
    let mut saadc_irq = interrupt::take!(SAADC);

    let server: Server = unwrap!(gatt_server::register(sd));

    // button presses will be delivered on HiToLo or when you release the button
    let button1 = gpiote::InputChannel::new(
        // degrade just a typesystem hack to forget which pin it is so we can
        // call it Anypin and make our function calls more generic
        dp.GPIOTE_CH1.degrade(),
        gpio::Input::new(dp.P0_00.degrade(), gpio::Pull::Down),
        gpiote::InputChannelPolarity::LoToHi,
    );

    // going to share these with multiple futures which will be created and
    // destroyed complicating lifetimes otherwise
    SERVER.borrow().replace(Some(server));

    #[rustfmt::skip]
    let adv_data = &[
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03, 0x03, 0x09, 0x18,
        0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't',
    ];

    #[rustfmt::skip]
    let scan_data = &[
        0x03, 0x03, 0x09, 0x18,
    ];

    let config = peripheral::Config::default();
    info!("Bluetooth is OFF");
    info!("Press puck.js button to enable, press again to disconnect");

    'waiting: loop {
        // wait here until button is pressed
        button1.wait().await;

        'advertising: loop {
            info!("advertising!");

            let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
                adv_data,
                scan_data,
            };

            let conn_future = peripheral::advertise_connectable(sd, adv, &config);

            // instead of await to run one future, well race several futures
            let conn = match select(button1.wait(), conn_future).await {
                // button returns if pressed stopping advertising and returning back to waiting state
                Either::First(_) => {
                    info!("stopping");
                    continue 'waiting;
                }
                // connection returns if somebody connects continuing execution
                Either::Second(conn) => unwrap!(conn),
            };

            let _green = gpio::Output::new(
                &mut dp.P0_04,
                gpio::Level::High,
                gpio::OutputDrive::Standard,
            );

            let mut blue =
                gpio::Output::new(&mut dp.P0_03, gpio::Level::Low, gpio::OutputDrive::Standard);

            info!("connected!");

            if let Some(server) = SERVER.borrow().borrow().as_ref() {
                // Run the GATT server on the connection. This returns when the connection gets disconnected.
                let gatt_future = gatt_server::run(&conn, server, |e| match e {
                    ServerEvent::BasService(e) => match e {
                        BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                            info!("battery notifications: {}", notifications)
                        }
                    },
                    ServerEvent::LedService(e) => match e {
                        LedServiceEvent::LedWrite(val) => {
                            if val > 0 {
                                blue.set_high()
                            } else {
                                blue.set_low()
                            }
                            info!("wrote led: {}", val);
                        }
                    },
                });

                match select3(battery_task(&mut saadc_irq), gatt_future, button1.wait()).await {
                    // battery never returns
                    Either3::First(_) => {}
                    // gatt returns if connection and goes stops advertising
                    Either3::Second(_) => continue 'waiting,
                    // button returns if pressed and stops advertising
                    Either3::Third(_) => continue 'waiting,
                }
            }
        }
    }
}

// Gain = (1/6) REFERENCE = (0.6 V or 6/10) RESOLUTION = 14bits
// Max InputV = (6/10)/(1/6) = 36/10 or 3600mv
// bat_min_mv = 2200
// VBAT_MV_PER_LSB = Max Input/ 2^RESOLUTION
// VBAT_MV_PER_LSB = 3600/16384
// mv = raw * VBAT_MV_PER_LSB
// mv = (3600 * raw)/16384

async fn battery_task(saadc_irq: &mut SAADC) {
    let mut dp = unsafe { <embassy_nrf::Peripherals as embassy::util::Steal>::steal() };
    loop {
        if let Some(server) = SERVER.borrow().borrow().as_ref() {
            let mut config = saadc::Config::default();
            // must change battery calculation if resolution changes
            config.resolution = saadc::Resolution::_14BIT;
            let channel_config = saadc::ChannelConfig::single_ended(saadc::VddInput);
            let mut saadc = Saadc::new(&mut dp.SAADC, &mut *saadc_irq, config, [channel_config]);

            let mut battery = [0; 1];
            saadc.sample(&mut battery).await;
            defmt::info!("{}", battery[0] as u32);

            let mv = battery[0] as u32 * 3600 / 16384;
            defmt::info!("{}mv", mv);

            let percentage = percent_from_mv::<2200, 3000>(mv);
            defmt::info!("{}%", percentage);

            unwrap!(server.bas_service.battery_level_set(percentage));
        }
        Timer::after(Duration::from_secs(60)).await;
    }
}

pub fn percent_from_mv<const MIN: u32, const MAX: u32>(mv: u32) -> u8 {
    debug_assert!(MAX > MIN);
    let mv = mv.min(MAX);
    let mv = mv.max(MIN + 1);
    let percent = (100 * (mv - MIN)) / (MAX - MIN);

    // SAFETY: has to be between 0 and 99
    percent as u8
}

#[embassy::task]
pub async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

pub fn softdevice_config() -> nrf_softdevice::Config {
    nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_250_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 32768,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 1,
            central_role_count: 1,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"HelloRust" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    }
}
