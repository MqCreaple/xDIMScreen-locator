use opencv::prelude::*;
use opencv::{imgproc, highgui, videoio};
use opencv::core::{Point, Scalar};
use xDIMScreen_locator::tag::apriltag::ImageU8View;
use xDIMScreen_locator::{self, tag::apriltag};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let window = "video";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
    let cam_index = 0;
    let mut cam = videoio::VideoCapture::new(cam_index, videoio::CAP_ANY)?;
	let opened = videoio::VideoCapture::is_opened(&cam)?;
	if !opened {
		return Err(format!("Unable to open camera at index {}", cam_index).into());
	}

    // create a tag detector with tag36h11 family
    let mut tag_family = apriltag::ApriltagFamilyType::new(apriltag::ApriltagFamily::Tag36h11);
    let mut detector = apriltag::ApriltagDetector::new();
    detector.add_family(&mut tag_family);

    loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        if frame.size()?.width <= 0 {
            continue;
        }

        let mut gray = Mat::default();
        imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0, opencv::core::AlgorithmHint::ALGO_HINT_ACCURATE)?;

        let mut image = ImageU8View::from(&mut gray);
        let detections = detector.detect(image.inner_mut());
        for detection in detections {
            // draw the detected apriltag on the frame
            for i in 0..4 {
                let start_pt = detection.corners()[i];
                let end_pt = detection.corners()[(i + 1) % 4];
                imgproc::line(
                    &mut frame,
                    Point::new(start_pt.0[0].round() as i32, start_pt.0[1].round() as i32),
                    Point::new(end_pt.0[0].round() as i32, end_pt.0[1].round() as i32),
                    Scalar::new(45., 44., 233., 0.),
                    2,
                    imgproc::LINE_8,
                    0,
                )?;
            }
        }
        highgui::imshow(window, &frame)?;

		let key = highgui::wait_key(30)?;
		if key > 0 && key != 255 {
			break;
		}
    }

    Ok(())
}