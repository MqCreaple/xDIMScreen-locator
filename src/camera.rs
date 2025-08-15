use std::sync::{
    Arc, RwLock,
    atomic::{AtomicBool, Ordering},
};
use std::thread::Thread;
use std::time::SystemTime;

use opencv::{
    core::{CV_64F, MatExpr, Vec4d},
    prelude::*,
    videoio,
};

extern crate nalgebra as na;

pub struct CameraProperty {
    pub resolution: (u32, u32),
    pub fov: (Option<f64>, Option<f64>),
    pub(crate) camera_mat: Mat,
    pub(crate) distortion: Mat,
}

impl CameraProperty {
    fn get_uncalibrated_camera_mat(
        resolution: (u32, u32),
        fov: (Option<f64>, Option<f64>),
    ) -> Result<Mat, Box<dyn std::error::Error>> {
        let (x_scaling, y_scaling) = match fov {
            (Some(fov_x), Some(fov_y)) => (f64::tan(fov_x * 0.5), f64::tan(fov_y * 0.5)),
            (Some(fov_x), None) => {
                let xs = f64::tan(fov_x * 0.5);
                (xs, xs * (resolution.1 as f64) / (resolution.0 as f64))
            }
            (None, Some(fov_y)) => {
                let ys = f64::tan(fov_y * 0.5);
                (ys * (resolution.0 as f64) / (resolution.1 as f64), ys)
            }
            (None, None) => {
                return Err("You must specify the FOV on either x or y component!".into());
            }
        };
        let half_resolution = ((resolution.0 as f64) * 0.5, (resolution.1 as f64) * 0.5);
        let uv_to_image_data = [
            half_resolution.0,
            0.0,
            half_resolution.0,
            0.0,
            half_resolution.1,
            half_resolution.1,
            0.0,
            0.0,
            1.0,
        ];
        let uv_to_image = Mat::new_rows_cols_with_data(3, 3, &uv_to_image_data)?;
        let world_to_uv_data = [
            1.0 / x_scaling,
            0.0,
            0.0,
            0.0,
            1.0 / y_scaling,
            0.0,
            0.0,
            0.0,
            1.0,
        ];
        let world_to_uv = Mat::new_rows_cols_with_data(3, 3, &world_to_uv_data)?;
        let ans =
            (MatExpr::from_mat(&uv_to_image)? * MatExpr::from_mat(&world_to_uv)?).into_result()?;
        Ok(ans.to_mat()?)
    }

    pub fn new(
        resolution: (u32, u32),
        fov: (Option<f64>, Option<f64>),
        camera_mat_and_distortion: Option<(Mat, Mat)>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let distortion = Mat::new_nd_with_default(&[5], CV_64F, Vec4d::all(0.0))?;
        let (camera_mat, distortion) = camera_mat_and_distortion
            .or(Some((
                Self::get_uncalibrated_camera_mat(resolution, fov)?,
                distortion,
            )))
            .ok_or("Cannot determine the camera matrix!")?;
        Ok(Self {
            resolution,
            fov,
            camera_mat,
            distortion,
        })
    }

    pub fn camera_mat(&self) -> &Mat {
        &self.camera_mat
    }

    pub fn camera_mat_na(&self) -> Result<na::Matrix3<f64>, Box<dyn std::error::Error>> {
        unsafe {
            Ok(na::Matrix3::new(
                *self.camera_mat.at_2d_unchecked(0, 0)?,
                *self.camera_mat.at_2d_unchecked(0, 1)?,
                *self.camera_mat.at_2d_unchecked(0, 2)?,
                *self.camera_mat.at_2d_unchecked(1, 0)?,
                *self.camera_mat.at_2d_unchecked(1, 1)?,
                *self.camera_mat.at_2d_unchecked(1, 2)?,
                *self.camera_mat.at_2d_unchecked(2, 0)?,
                *self.camera_mat.at_2d_unchecked(2, 1)?,
                *self.camera_mat.at_2d_unchecked(2, 2)?,
            ))
        }
    }
}

pub fn camera_thread_main(
    termination_signal: Arc<AtomicBool>,
    mut cam: videoio::VideoCapture,
    shared_frame: Arc<RwLock<(Mat, SystemTime)>>,
    parked_threads: Vec<&Thread>,
) -> Result<(), Box<dyn std::error::Error>> {
    while !termination_signal.load(Ordering::Relaxed) {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        if frame.size()?.width <= 0 {
            continue;
        }
        let mut shared_frame_write = shared_frame.write().unwrap();
        *shared_frame_write = (frame, SystemTime::now());
        drop(shared_frame_write);
        for thread in &parked_threads {
            thread.unpark();
        }
    }

    // Make sure all threads are unparked
    for thread in &parked_threads {
        thread.unpark();
    }
    Ok(())
}
