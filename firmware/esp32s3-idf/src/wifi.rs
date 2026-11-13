#![cfg(feature = "webserver")]

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    watch::{Sender, Watch},
};
use embassy_time::Timer;
use esp_idf_hal::modem::Modem;
use esp_idf_svc::{
    eventloop::{EspEventLoop, System},
    nvs::{EspNvsPartition, NvsDefault},
    timer::{EspTimerService, Task},
    wifi::{AccessPointConfiguration, AsyncWifi, AuthMethod, Configuration, EspWifi},
};

use dotenvy_macro::dotenv;
use log::*;

pub static WIFI_STATE: Watch<Cs, WifiState, 1> = Watch::new();

#[derive(Clone)]
pub enum WifiState {
    Disconnected,
    Connected,
}

#[embassy_executor::task]
pub async fn wifi_task(
    mut modem: Modem,
    sys_loop: EspEventLoop<System>,
    nvs: EspNvsPartition<NvsDefault>,
    timer_service: EspTimerService<Task>,
    wifi_state_sender: Sender<'static, Cs, WifiState, 1>,
) {
    log::info!("Setting up Wifi stack");

    loop {
        log::info!("Trying to connect to Wifi");
        match try_connect(
            // dotenv!("WIFI_SSID"),
            // dotenv!("WIFI_PASSWORD"),
            &mut modem,
            sys_loop.clone(),
            nvs.clone(),
            timer_service.clone(),
        )
        .await
        {
            Ok(wifi) => {
                log::info!("Wifi connected");

                wifi_state_sender.send(WifiState::Connected);

                // Ghetto connectivity check
                loop {
                    if !wifi.is_connected().unwrap_or(false) {
                        warn!("Wifi connection dropped!");
                        break;
                    }

                    Timer::after_millis(1000).await;
                }
            }
            Err(err) => {
                warn!("Wifi unable to connect: {err}, retrying...");
            }
        }
    }
}

async fn try_connect<'a>(
    // ssid: &'static str,
    // password: &'static str,
    modem: &'a mut Modem,
    sys_loop: EspEventLoop<System>,
    nvs: EspNvsPartition<NvsDefault>,
    timer_service: EspTimerService<Task>,
) -> anyhow::Result<AsyncWifi<EspWifi<'a>>> {
    let mut wifi = AsyncWifi::wrap(
        EspWifi::new(modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
        timer_service,
    )?;

    unsafe { esp_idf_sys::esp_wifi_set_country_code(b"EU\0".as_ptr() as _, true) };

    let mut ap_cfg = AccessPointConfiguration::default();
    ap_cfg.ssid = "peeler".try_into().unwrap();
    ap_cfg.channel = 6; // wifi channel
    let configuration = Configuration::AccessPoint(ap_cfg);

    wifi.set_configuration(&configuration)?;

    wifi.start().await?;
    log::info!("Wifi started");

    unsafe {
        // 84 * 0.25 dBm = 21 dBm requested; the driver clamps to what the chip/regulatory domain allows
        esp_idf_sys::esp!(esp_idf_sys::esp_wifi_set_max_tx_power(84))?;

        // Read it back to see what you actually got
        let mut power: i8 = 0;
        esp_idf_sys::esp!(esp_idf_sys::esp_wifi_get_max_tx_power(&mut power))?;
        log::info!("TX power: {} ({} dBm)", power, power as f32 * 0.25);
    }

    wifi.wait_netif_up().await?;
    log::info!("Wifi netif up");

    log::info!("Device ip: {}", wifi.wifi().ap_netif().get_ip_info()?.ip);
    Ok(wifi)
}
