use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;

pub fn handle_error(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let uri = request.uri().to_string();
    log::warn!("Triggered error handler for {uri}");
    let mut resp = request.into_response(
        503,
        Some("Service Unavailable"),
        &[("Content-Type", "text/plain")],
    )?;
    resp.write_all(format!("Errorhandler triggered for {uri}").as_bytes())?;
    resp.flush()?;
    Ok(())
}
