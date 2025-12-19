use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread;
use std::time::SystemTime;

use opencv::imgproc;
use opencv::prelude::*;
#[cfg(feature = "visualize")]
use opencv::{core, highgui};

use crate::tag::apriltag::ImageU8View;

extern crate nalgebra as na;

/// A utility module for binding to the apriltag C library
pub mod apriltag;

/// Defines the tagged 3D objects for spatial locating
pub mod tagged_object;

/// Code related to locating tagged object with computer vision
pub mod locator;

/// Defines the errors related to object tagging and tag recognitions
pub mod error;

pub fn locator_thread_main<'a>(
    termination_signal: Arc<AtomicBool>,
    shared_frame: Arc<RwLock<(Mat, SystemTime)>>,
    detector: apriltag::ApriltagDetector,
    mut object_locator: locator::TaggedObjectLocator<'a>,
    located_objects: Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>,
) -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(feature = "visualize")]
    let object_map = object_locator.get_object_map();
    #[cfg(feature = "visualize")]
    let camera_mat = object_locator.camera().camera_mat_na()?;

    #[cfg(feature = "visualize")]
    highgui::named_window("window", highgui::WINDOW_KEEPRATIO)?;

    let mut last_recorded_timestamp = SystemTime::UNIX_EPOCH;
    while !termination_signal.load(Ordering::Relaxed) {
        let mut shared_frame_mat = loop {
            // park the thread and wait for the camera thread to unpark it
            thread::park();
            // when unparked, read the camera frame
            let shared_frame_read = shared_frame.read().unwrap();
            if shared_frame_read.1 != last_recorded_timestamp {
                // check the timestamp to prevent false unparking
                last_recorded_timestamp = shared_frame_read.1;
                break shared_frame_read.0.clone();
            }
        };
        // to ensure that the timestamp accurately reflects the time at which
        // the objects are located.
        let mut gray = Mat::default();
        imgproc::cvt_color(
            &shared_frame_mat,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_ACCURATE,
        )?;
        let mut image = ImageU8View::from(&mut gray);
        let detections = detector.detect(image.inner_mut());

        object_locator.locate_objects(
            last_recorded_timestamp,
            detections.as_slice(),
            located_objects.clone(),
        )?;

        #[cfg(feature = "visualize")]
        {
            use crate::tag::locator::TAG_CORNERS;

            // draw the detected apriltag on the frame
            for detection in &detections {
                for i in 0..4 {
                    let start_pt = detection.corners()[i];
                    let end_pt = detection.corners()[(i + 1) % 4];
                    imgproc::line(
                        &mut shared_frame_mat,
                        core::Point::new(start_pt.x.round() as i32, start_pt.y.round() as i32),
                        core::Point::new(end_pt.x.round() as i32, end_pt.y.round() as i32),
                        core::Scalar::new(
                            if i != 0 { 255. } else { 0. },
                            if i != 1 { 255. } else { 0. },
                            if i != 2 { 255. } else { 0. },
                            0.,
                        ),
                        2,
                        imgproc::LINE_8,
                        0,
                    )?;
                }
            }
            // draw each tag's reprojection on the image
            let lock = located_objects.0.lock().unwrap();
            for (name, loc) in lock.name_map() {
                if let Some(object) = object_map.get(*name) {
                    let color = crate::visualize::utils::generate_random_color(name);
                    // plot the reprojection of all tags
                    for (_, tag_loc) in object {
                        let corners = TAG_CORNERS
                            .iter()
                            .map(|point| {
                                let point1 = tag_loc.0.transform_point(point);
                                let point2 = loc.transform_point(&point1);
                                let projected = camera_mat * point2;
                                projected.xy() / projected.z
                            })
                            .collect::<Vec<_>>();
                        for i in 0..4 {
                            imgproc::line(
                                &mut shared_frame_mat,
                                core::Point::new(corners[i].x as i32, corners[i].y as i32),
                                core::Point::new(corners[i + 1].x as i32, corners[i + 1].y as i32),
                                core::Scalar::new(
                                    color.2 as f64,
                                    color.1 as f64,
                                    color.0 as f64, // in the order of BGR
                                    0.0,
                                ),
                                2,
                                imgproc::LINE_8,
                                0,
                            )?;
                        }
                        // plot the x, y, and z axes of each tag
                        for i in 0..3 {
                            use crate::visualize::utils::{AXES, AXES_COLORS};

                            let axis_origin = camera_mat
                                * loc.transform_point(
                                    &tag_loc.0.transform_point(&na::Point3::origin()),
                                );
                            let axis_origin = axis_origin.xy() / axis_origin.z;
                            let axis_end = camera_mat
                                * loc.transform_point(&tag_loc.0.transform_point(&AXES[i]));
                            let axis_end = axis_end.xy() / axis_end.z;
                            let color = AXES_COLORS[i];
                            imgproc::line(
                                &mut shared_frame_mat,
                                core::Point::new(axis_origin.x as i32, axis_origin.y as i32),
                                core::Point::new(axis_end.x as i32, axis_end.y as i32),
                                core::Scalar::new(
                                    color.2 as f64,
                                    color.1 as f64,
                                    color.0 as f64,
                                    0.0,
                                ), // in the order of BGR
                                2,
                                imgproc::LINE_8,
                                0,
                            )?;
                        }
                    }
                }
            }
            drop(lock);
            // show image
            highgui::imshow("window", &shared_frame_mat)?;
            // wait for exit key
            let key = highgui::wait_key(1)?;
            if key > 0 && key != 255 {
                break;
            }
        }
    }
    Ok(())
}
