use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Sender};
use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;
use messenger_mouse::Esp32Setpoint;

use crate::request::ReadableRequest;

pub fn handle_setpoint(
    mut request: Request<&mut EspHttpConnection<'_>>,
    sender: &Sender<'static, Cs, Esp32Setpoint, 1>,
) -> anyhow::Result<()> {
    log::info!("Received setpoint");

    // Try to deserialize received data into setpoint
    let readable_request = ReadableRequest(&mut request);
    let setpoint: Esp32Setpoint = match readable_request.deserialize_into() {
        Ok(r) => r,
        Err(err) => {
            log::warn!("Unable to deserialize get request into setpoint: {err}",);
            let mut response = request.into_response(
                400,
                Some("Bad Request"),
                &[("Content-Type", "text/plain")],
            )?;
            response.write_all(format!("Invalid JSON: {err}").as_bytes())?;
            response.flush()?;
            return Ok(());
        }
    };
    log::info!("Deserialisation success! {setpoint:?}");

    sender.send(setpoint);

    // Success response
    let mut response = request.into_response(200, Some("OK"), &[("Content-Type", "text/plain")])?;
    response.write_all(b"Depth setpoint updated")?;
    response.flush()?;
    Ok(())
}
