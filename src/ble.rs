use core::cell::RefCell;
use defmt::{info, unwrap};
use embassy::blocking_mutex::ThreadModeMutex;
use embassy::time::{Duration, Timer};
use embassy_nrf::gpio::{self, AnyPin};
use embassy_nrf::gpiote::{AnyChannel, InputChannel};
use embassy_nrf::saadc;
use embedded_hal::digital::v2::OutputPin;
use futures::FutureExt;
use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};
use panic_probe as _;

// define a bluetooth service with one characteristic we can write and read to
#[nrf_softdevice::gatt_service(uuid = "9e7312e0-2354-11eb-9f10-fbc30a62cf38")]
struct MyService {
    #[characteristic(uuid = "9e7312e0-2354-11eb-9f10-fbc30a63cf38", read, write)]
    my_char: u8,
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_server]
struct Server {
    bas_service: BatteryService,
    my_service: MyService,
}

// tasks in same executor guaranteed to not be running at same time as they cant interupt eachother
static SERVER: ThreadModeMutex<RefCell<Option<Server>>> = ThreadModeMutex::new(RefCell::new(None));

static SAADC: ThreadModeMutex<RefCell<Option<saadc::Saadc<'static, 1>>>> =
    ThreadModeMutex::new(RefCell::new(None));

static GREEN_LED: ThreadModeMutex<RefCell<Option<gpio::Output<'static, AnyPin>>>> =
    ThreadModeMutex::new(RefCell::new(None));

#[embassy::task]
pub async fn bluetooth_task(
    sd: &'static Softdevice,
    button1: InputChannel<'static, AnyChannel, AnyPin>,
    mut blue: gpio::Output<'static, AnyPin>,
    green: gpio::Output<'static, AnyPin>,
    saadc: saadc::Saadc<'static, 1>,
) {
    let server: Server = unwrap!(gatt_server::register(sd));

    // going to share these with multiple futures which will be created and
    // destroyed complicating lifetimes otherwise
    SERVER.borrow().replace(Some(server));
    GREEN_LED.borrow().replace(Some(green));
    SAADC.borrow().replace(Some(saadc));

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
        // set green led off
        if let Some(green) = GREEN_LED.borrow().borrow_mut().as_mut() {
            unwrap!(green.set_low())
        }

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
            let conn = futures::select_biased! {
                // blink led to show advertising status, doesnt actually return
                _ = blinky_task().fuse() => continue 'waiting,
                // button returns if pressed stopping advertising and returning back to waiting state
                _ = button1.wait().fuse() => {info!("stopping"); continue 'waiting;},
                // connection returns if somebody connects continuing execution
                conn = conn_future.fuse() => unwrap!(conn),
            };

            // enable green led to indicate a connection
            if let Some(green) = GREEN_LED.borrow().borrow_mut().as_mut() {
                unwrap!(green.set_high())
            }

            info!("connected!");

            if let Some(server) = SERVER.borrow().borrow().as_ref() {
                // Run the GATT server on the connection. This returns when the connection gets disconnected.
                let gatt_future = gatt_server::run(&conn, server, |e| match e {
                    ServerEvent::BasService(e) => match e {
                        BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                            info!("battery notifications: {}", notifications)
                        }
                    },
                    ServerEvent::MyService(e) => match e {
                        MyServiceEvent::MyCharWrite(val) => {
                            if val > 0 {
                                unwrap!(blue.set_low());
                            } else {
                                unwrap!(blue.set_high());
                            }
                            info!("wrote my_char: {}", val);
                        }
                    },
                });

                futures::select_biased! {
                    // battery never returns
                    _ = battery_task().fuse() => (),
                    // gatt returns if connection and goes back to advertising
                    _ = gatt_future.fuse() => continue 'advertising,
                    // button returns if pressed and stops advertising
                    _ = button1.wait().fuse() => continue 'waiting,
                };
            }
        }
    }
}

async fn blinky_task() {
    loop {
        if let Some(green) = GREEN_LED.borrow().borrow_mut().as_mut() {
            unwrap!(green.set_high())
        }
        Timer::after(Duration::from_millis(1000)).await;

        if let Some(green) = GREEN_LED.borrow().borrow_mut().as_mut() {
            unwrap!(green.set_low())
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

// Gain = (1/6) REFERENCE = (0.6 V) RESOLUTION = 14bits
// Max Input = (0.6 V)/(1/6) = 3.6 V
// ADC RESULT = [V(p)- V(n)] * (GAIN/REFERENCE) * 2^(RESOLUTION)
// ADC RESULT = [V(p) - 0] * 2^(RESOLUTION) * (GAIN/REFERENCE)
// ADC RESULT = V(p) * 2^(RESOLUTION) * (GAIN/REFERENCE)
// ADC RESULT = V(p) * 16384 * (1/6)/(3/5)
// ADC RESULT = V(p) * 16384 * (5/18)
// ADC RESULT = V(p) * 81920 / 18
// V(p) = ADC RESULT * 18 / 81920
// Percentage = V(p) * 100 / Max Input
// Percentage = (ADC RESULT * 1800 / 81920 ) / 3.6
// Percentage = ADC RESULT * 500 / 81920
async fn battery_task() {
    loop {
        if let (Some(server), Some(saadc)) = (
            SERVER.borrow().borrow().as_ref(),
            SAADC.borrow().borrow_mut().as_mut(),
        ) {
            let mut battery = [0; 1];
            saadc.sample(&mut battery).await;

            let percentage = (battery[0] as u32 * 500 / 81920) as u8;

            defmt::info!("{}%", percentage);

            unwrap!(server.bas_service.battery_level_set(percentage));
        }
        Timer::after(Duration::from_secs(60)).await;
    }
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
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_20_PPM as u8,
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
