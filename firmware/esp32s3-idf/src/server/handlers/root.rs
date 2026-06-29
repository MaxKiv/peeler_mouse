use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;

#[cfg(not(feature = "streaming"))]
pub fn handle_root(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    // Medium levels of cursed
    let html = include_str!("./root.html");

    let content_length = html.len().to_string();
    let headers = [
        ("Content-Type", "text/html"),
        ("Content-Length", content_length.as_str()),
        ("Connection", "close"),
    ];

    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(html.as_bytes())?;
    response.flush()?;
    Ok(())
}
