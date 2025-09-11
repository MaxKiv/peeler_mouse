pub mod espcam;
pub mod wifi;

use std::{ffi::CStr, sync::Arc};

use anyhow::Result;
use dotenvy_macro::dotenv;
use embassy_executor::Spawner;
use esp_idf_hal::{io::Write, prelude::Peripherals};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    http::{server::EspHttpServer, Method},
    nvs::EspDefaultNvsPartition,
    timer::EspTaskTimerService,
};

use crate::espcam::Camera;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    if let Err(err) = main_fallible(&spawner).await {
        log::error!("MAIN: {err}");
    }
}

async fn main_fallible(spawner: &Spawner) -> Result<()> {
    let _ = spawner;
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let version = unsafe { esp_idf_sys::esp_get_idf_version() };
    let version = unsafe { CStr::from_ptr(version) };
    let version = version.to_str()?;
    log::info!("ESP-IDF version: {version}");

    log::info!("Setting up peripherals, esp event loop, nvs partition and timer service");
    let mut peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = EspTaskTimerService::new()?;

    log::info!("Setting up Wifi stack");
    let wifi = wifi::connect_to(
        dotenv!("WIFI_SSID"),
        dotenv!("WIFI_PASSWORD"),
        &mut peripherals.modem,
        sys_loop.clone(),
        nvs.clone(),
        timer_service.clone(),
    )
    .await?;

    log::info!("Device ip: {}", wifi.wifi().ap_netif().get_ip_info()?.ip);

    log::info!("Setting up camera");
    let camera = Camera::new(
        peripherals.pins.gpio32,
        peripherals.pins.gpio0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        peripherals.pins.gpio21,
        peripherals.pins.gpio36,
        peripherals.pins.gpio39,
        peripherals.pins.gpio34,
        peripherals.pins.gpio35,
        peripherals.pins.gpio25,
        peripherals.pins.gpio23,
        peripherals.pins.gpio22,
        peripherals.pins.gpio26,
        peripherals.pins.gpio27,
        esp_idf_sys::camera::pixformat_t_PIXFORMAT_JPEG,
        // Set quality here
        esp_idf_sys::camera::framesize_t_FRAMESIZE_SVGA,
    )?;

    let cam_arc = Arc::new(camera);
    let cam_arc_clone = cam_arc.clone();

    let mut server = EspHttpServer::new(&esp_idf_svc::http::server::Configuration::default())?;

    server.fn_handler(
        "/camera",
        Method::Get,
        move |request| -> Result<(), anyhow::Error> {
            let part_boundary = "123456789000000000000987654321";
            let frame_boundary = format!("\r\n--{part_boundary}\r\n");

            let content_type = format!("multipart/x-mixed-replace;boundary={part_boundary}");
            let headers = [("Content-Type", content_type.as_str())];
            let mut response = request.into_response(200, Some("OK"), &headers).unwrap();
            loop {
                if let Some(fb) = cam_arc_clone.get_framebuffer() {
                    let data = fb.data();
                    let frame_part = format!(
                        "Content-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                        data.len()
                    );
                    response.write_all(frame_part.as_bytes())?;
                    response.write_all(data)?;
                    response.write_all(frame_boundary.as_bytes())?;
                    response.flush()?;
                }
            }

            Ok(())
        },
    )?;

    server.fn_handler("/", Method::Get, |request| -> Result<(), anyhow::Error> {
        let data = "<html><head><meta name=\"viewport\" content=\"width=device-width; height=device-height;\"><title>esp32cam</title></head><body><img src=\"camera\" alt=\"Failed to load image\" style=\"height: 100%;width: 100%; transform: rotate(180deg);\"></body></html>";


        let headers = [
            ("Content-Type", "text/html"),
            ("Content-Length", &data.len().to_string()),
        ];
        let mut response = request.into_response(200, Some("OK"), &headers)?;
        response.write_all(data.as_bytes())?;
        Ok(())
    })?;

    core::future::pending::<()>().await;

    Ok(())
}
