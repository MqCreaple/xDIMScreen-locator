use opencv::core::Point3_;
use opencv::prelude::*;
use opencv::{Result, highgui, videoio};

fn main() -> Result<()> {
    let window1 = "video capture";
    let window2 = "thresholded";
    highgui::named_window(window1, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(window2, highgui::WINDOW_AUTOSIZE)?;
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    let opened = videoio::VideoCapture::is_opened(&cam)?;
    if !opened {
        panic!("Unable to open default camera!");
    }
    loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        if frame.size()?.width > 0 {
            highgui::imshow(window1, &frame)?;
            // round each pixel's RGB color to either 0 or 255
            for (_i, elem) in frame.iter_mut::<Point3_<u8>>()? {
                elem.x = if elem.x < 128 { 0 } else { 255 };
                elem.y = if elem.y < 128 { 0 } else { 255 };
                elem.z = if elem.z < 128 { 0 } else { 255 };
            }
            highgui::imshow(window2, &frame)?;
        }
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    Ok(())
}
