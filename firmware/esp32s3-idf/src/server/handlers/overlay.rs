use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};

use crate::{
    camera::{
        framebuffer_view::{BoundingBox, Pixel},
        FRAME_SIZE,
    },
    control::{control_loop::body::VISION_OUTPUT_WATCH, vision::algo::VISION_BOUNDING_BOX},
};

#[derive(Default)]
pub struct OutputOverlayData {
    pub zero_line: (Pixel, Pixel),
    pub transition_line: (Pixel, Pixel),
    pub bounding_box: BoundingBox,
    pub tearing_detected: i32,
}

pub fn handle_overlay(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let mut rx = VISION_OUTPUT_WATCH
        .receiver()
        .expect("not enough VISION_OUTPUT_WATCH rx N");

    // Get latest vision output
    let vision_output = rx.try_get().unwrap_or_default();
    let overlay = OutputOverlayData {
        zero_line: (
            Pixel {
                x: 0,
                y: vision_output.zero_line_height_px as usize,
            },
            Pixel {
                x: FRAME_SIZE.get_dimensions().0,
                y: vision_output.zero_line_height_px as usize,
            },
        ),
        transition_line: (
            Pixel {
                x: 0,
                y: vision_output.transition_line_height_px.unwrap_or_default() as usize,
            },
            Pixel {
                x: FRAME_SIZE.get_dimensions().0,
                y: vision_output.transition_line_height_px.unwrap_or_default() as usize,
            },
        ),
        bounding_box: VISION_BOUNDING_BOX,
        tearing_detected: vision_output.tearing_detected as i32,
    };

    // Format into JSON
    let json = format!(
        r#"{{"zero_line":[{},{},{},{}],"transition_line":[{},{},{},{}], "bb":[{},{},{},{}], "tearing_detected": {}}}"#,
        overlay.zero_line.0.x,
        overlay.zero_line.0.y,
        overlay.zero_line.1.x,
        overlay.zero_line.1.y,
        overlay.transition_line.0.x,
        overlay.transition_line.0.y,
        overlay.transition_line.1.x,
        overlay.transition_line.1.y,
        overlay.bounding_box.start.x,
        overlay.bounding_box.start.y,
        overlay.bounding_box.width,
        overlay.bounding_box.height,
        overlay.tearing_detected,
    );

    let headers = [
        ("Content-Type", "application/json"),
        ("Cache-Control", "no-store"),
    ];
    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(json.as_bytes())?;
    response.flush()?;
    Ok(())
}
