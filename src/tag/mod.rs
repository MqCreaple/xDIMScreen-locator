use std::sync::{Arc, Condvar, Mutex};
use std::time::SystemTime;

#[cfg(feature = "visualize")]
use opencv::highgui;
use opencv::prelude::*;
use opencv::{core, imgproc, videoio};

use crate::tag::apriltag::ImageU8View;

/// A utility module for binding to the apriltag C library
pub mod apriltag;

/// Defines the tagged 3D objects for spatial locating
pub mod tagged_object;

/// Code related to locating tagged object with computer vision
pub mod locator;

/// Defines the errors related to object tagging and tag recognitions
pub mod error;

pub fn locator_thread_main<'a>(
    mut cam: videoio::VideoCapture,
    detector: apriltag::ApriltagDetector,
    mut object_locator: locator::TaggedObjectLocator<'a>,
    located_objects: Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        if frame.size()?.width <= 0 {
            continue;
        }
        let timestamp = SystemTime::now(); // Measure the timestamp right after acquiring a frame from camera
        // to ensure that the timestamp accurately reflects the time at which
        // the objects are located.
        let mut gray = Mat::default();
        imgproc::cvt_color(
            &frame,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_ACCURATE,
        )?;
        let mut image = ImageU8View::from(&mut gray);
        let detections = detector.detect(image.inner_mut());

        #[cfg(feature = "visualize")]
        {
            for detection in &detections {
                // draw the detected apriltag on the frame
                for i in 0..4 {
                    let start_pt = detection.corners()[i];
                    let end_pt = detection.corners()[(i + 1) % 4];
                    imgproc::line(
                        &mut frame,
                        core::Point::new(start_pt.x.round() as i32, start_pt.y.round() as i32),
                        core::Point::new(end_pt.x.round() as i32, end_pt.y.round() as i32),
                        core::Scalar::new(45., 44., 233., 0.),
                        2,
                        imgproc::LINE_8,
                        0,
                    )?;
                }
            }
            highgui::imshow("window", &frame)?;
        }

        object_locator.locate_objects(timestamp, detections.as_slice(), located_objects.clone())?;

        #[cfg(feature = "visualize")]
        {
            // wait for exit key
            let key = highgui::wait_key(1)?;
            if key > 0 && key != 255 {
                break;
            }
        }
    }
    Ok(())
}
