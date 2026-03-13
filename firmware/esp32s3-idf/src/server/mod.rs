#![cfg(feature = "webserver")]

pub mod handlers;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use esp_idf_svc::http::{server::EspHttpServer, Method};
use log::*;

use crate::{
    camera::{pixelformat::PixelFormat, PIXEL_FORMAT},
    server::handlers::{
        camera::{grayscale::handle_camera_grayscale, jpeg::handle_camera_jpeg},
        error::handle_error,
        root::handle_root,
    },
    wifi::WifiState,
};

#[embassy_executor::task]
pub async fn server_task(mut wifi_state_receiver: Receiver<'static, Cs, WifiState, 1>) {
    log::info!("Starting Server Embassy task");

    loop {
        // Set up a HTTP server when wifi is connected
        match wifi_state_receiver.try_get() {
            Some(WifiState::Connected) => {
                match EspHttpServer::new(&esp_idf_svc::http::server::Configuration::default()) {
                    Ok(mut server) => {
                        info!("HTTP server constructed, setting up handlers");

                        // Set up HTTP server handlers
                        if let Err(err) = server.fn_handler("/", Method::Get, handle_root) {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }

                        if let Err(err) =
                            server.fn_handler("/camera", Method::Get, move |request| {
                                use PixelFormat::*;
                                match PIXEL_FORMAT {
                                    GRAYSCALE => handle_camera_grayscale(request),
                                    JPEG => handle_camera_jpeg(request),
                                    _ => handle_error(request),
                                }
                            })
                        {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }

                        // let sender = setpoint_sender.clone();
                        // if let Err(err) =
                        //     server.fn_handler("/setpoint", Method::Post, move |request| {
                        //         handle_setpoint(request, &sender)
                        //     })
                        // {
                        //     error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        // }

                        info!(
                            "HTTP server handlers set up, keeping alive until wifi connected is dropped"
                        );

                        // Keep server alive untill wifi connection drops
                        if let WifiState::Disconnected = wifi_state_receiver.changed().await {
                            warn!("Wifi disconnected, dropping & reconfiguring HTTP server");
                        }
                    }
                    Err(err) => {
                        error!("Unable to set up HTTP Server: {err}, retrying...");
                    }
                }
            }
            _ => {
                warn!("Wifi is not yet connected -> Can't set up webserver, retrying soon...");
                embassy_time::Timer::after_millis(500).await;
            }
        }
    }
}
