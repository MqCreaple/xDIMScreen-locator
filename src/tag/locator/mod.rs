use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, SystemTime};

use opencv::calib3d;
use opencv::prelude::*;

extern crate nalgebra as na;

use crate::camera::CameraProperty;
use crate::tag::apriltag;
use crate::tag::error::ConflictingTagError;
use crate::tag::tagged_object::{TagIndex, TagLocation, TaggedObject};
use crate::utils::rotation_jacobian;

/// A square tag's four corners in its local reference frame.
///
/// This array's order is kept consistent with Apriltag and OpenCV library.
pub const TAG_CORNERS: [na::Point3<f64>; 5] = [
    na::Point3::new(-1.0, 1.0, 0.0),
    na::Point3::new(1.0, 1.0, 0.0),
    na::Point3::new(1.0, -1.0, 0.0),
    na::Point3::new(-1.0, -1.0, 0.0),
    na::Point3::new(-1.0, 1.0, 0.0), // add the first point again to make drawing the tag easier
];

/// The duration after which an object's stored information is forgotten.
pub const OBJECT_FORGET_DURATION: Duration = Duration::from_secs(1);

pub struct TaggedObjectLocator<'a> {
    /// Camera matrix
    camera: CameraProperty,

    /// List of all objects registered in the object locator
    registry: Vec<&'a TaggedObject>,

    /// Mapping from each tag's property to its corresponding object's index in the registry array
    tag_map: HashMap<TagIndex, (usize, TagLocation)>,

    /// Each object's last location. These are used as the extrinsic guess for OpenCV's solvePnP function.
    ///
    /// This array's index corresponds to the objects stored in `registry`.
    last_location: Vec<Option<(Mat, Mat, SystemTime)>>,
}

/// A data struct for storing the located objects in each frame.
///
/// Lifetime parameter `'a` denotes the lifetime of the objects it is referring to. In other words, the
/// specific objects (e.g. handheld screen, wand, etc.) must live longer than the `LocatedObjects` referring
/// to them.
#[derive(Debug)]
pub struct LocatedObjects<'a> {
    pub(super) timestamp: SystemTime,
    pub(super) name_map: BTreeMap<&'a str, na::Isometry3<f64>>,
}

impl<'a> LocatedObjects<'a> {
    pub fn new() -> Self {
        Self {
            timestamp: SystemTime::now(),
            name_map: BTreeMap::new(),
        }
    }

    pub fn timestamp(&self) -> SystemTime {
        self.timestamp
    }

    pub fn name_map(&self) -> &BTreeMap<&'a str, na::Isometry3<f64>> {
        &self.name_map
    }
}

impl<'a> TaggedObjectLocator<'a> {
    pub fn new(camera: CameraProperty) -> Self {
        Self {
            camera,
            registry: Vec::new(),
            tag_map: HashMap::new(),
            last_location: Vec::new(),
        }
    }

    pub fn camera(&self) -> &CameraProperty {
        &self.camera
    }

    /// Add a new tagged object to the registry.
    pub fn add(&mut self, tagobj: &'a TaggedObject) -> Result<(), ConflictingTagError> {
        let this_name = &tagobj.name;
        for (tag_index, _) in &tagobj.tags {
            if let Some((registry_index, _)) = self.tag_map.get(tag_index) {
                return Err(ConflictingTagError::new(
                    *tag_index,
                    unsafe { self.registry.get_unchecked(*registry_index).name.clone() },
                    this_name.clone(),
                ));
            }
        }
        let this_registry_index = self.registry.len();
        self.registry.push(tagobj);
        for (tag_index, tag_location) in &tagobj.tags {
            // It is guaranteed that at this point, there's no conflict in tag indices
            self.tag_map
                .insert(*tag_index, (this_registry_index, tag_location.clone()));
        }
        self.last_location.push(None);
        Ok(())
    }

    pub fn get_object_map(&self) -> HashMap<String, Vec<(TagIndex, TagLocation)>> {
        self.registry
            .iter()
            .map(|obj| {
                (
                    obj.name.clone(),
                    obj.tags
                        .iter()
                        .map(|(i, loc)| (i.clone(), loc.clone()))
                        .collect::<Vec<_>>(),
                )
            })
            .collect()
    }

    /// Locate a single tag with OpenCV's SOLVEPNP_IPPE_SQUARE method.
    ///
    /// # Arguments
    /// * `detection` - The detection of the tag to locate.
    /// * `scale` - The scaling factor to multiply on the TAG_CORNERS array. This equals half of the tag's
    ///             side length.
    ///
    /// # Returns
    /// The function returns the transformation of the tag from the camera's center.
    fn locate_tag(
        &self,
        detection: &apriltag::ApriltagDetection,
        scale: f64,
    ) -> Result<na::Isometry3<f64>, Box<dyn std::error::Error>> {
        let mut object_points_data = [0.0f64; 12]; // `detections.len()` (tags) * `4` (vertices / tag) * `3` (coordinates / vertex)
        let mut image_points_data = [0.0f64; 8];
        for (i, corner) in detection.corners().iter().enumerate() {
            object_points_data[i * 3] = TAG_CORNERS[i].x * scale;
            object_points_data[i * 3 + 1] = TAG_CORNERS[i].y * scale;
            object_points_data[i * 3 + 2] = TAG_CORNERS[i].z * scale;
            image_points_data[i * 2] = corner.x;
            image_points_data[i * 2 + 1] = corner.y;
        }
        let object_points = Mat::new_rows_cols_with_data(4, 3, &object_points_data)?;
        let image_points = Mat::new_rows_cols_with_data(4, 2, &image_points_data)?;
        let mut rvec = Mat::default();
        let mut tvec = Mat::default();

        calib3d::solve_pnp(
            &object_points,
            &image_points,
            &self.camera.camera_mat,
            &self.camera.distortion,
            &mut rvec,
            &mut tvec,
            false,
            calib3d::SOLVEPNP_IPPE_SQUARE,
        )?;

        let rvec = unsafe {
            na::Vector3::new(
                *rvec.at_unchecked::<f64>(0),
                *rvec.at_unchecked::<f64>(1),
                *rvec.at_unchecked::<f64>(2),
            )
        };
        let tvec = unsafe {
            na::Vector3::new(
                *tvec.at_unchecked::<f64>(0),
                *tvec.at_unchecked::<f64>(1),
                *tvec.at_unchecked::<f64>(2),
            )
        };
        Ok(na::Isometry3::new(tvec, rvec))
    }

    /// Locate a single object based on the detected tag locations.
    ///
    /// # Arguments
    /// * `detections` - Stores a list of pairs of apriltag detections with their relative transformation
    ///                  from the object's center. This is created by filtering out the tags belonging to
    ///                  the object of interest from all tag detections in one frame.
    /// * `object_index` - The object's index in the `registry` array. If `object_id` is `None`, then the
    ///                    returned rotation and translation vectors won't be stored.
    /// * `timestamp` - The timestamp when the object location occurs.
    ///
    /// # Returns
    /// The function returns the transformation of the object's center in the camera's frame, or throw an
    /// error.
    fn locate_single_object<'b, 'c>(
        &mut self,
        detections: &'b [(&'c apriltag::ApriltagDetection, TagLocation)],
        object_index: Option<usize>,
        timestamp: SystemTime,
    ) -> Result<na::Isometry3<f64>, Box<dyn std::error::Error>> {
        let mut rvec = Mat::default();
        let mut tvec = Mat::default();
        // load the object's last location
        let mut use_extrinsic_guess = false;
        if let Some(object_index) = object_index
            && let Some(last_location) = &self.last_location[object_index]
        {
            if timestamp.duration_since(last_location.2)? <= OBJECT_FORGET_DURATION {
                // The object is not forgotten. Load `rvec` and `tvec` from `last_location`.
                rvec = last_location.0.clone();
                tvec = last_location.1.clone();
                use_extrinsic_guess = true;
            }
        }

        if detections.len() == 1 {
            // Only one tag is present. Use `locate_tag` function to achieve better performance.
            let (detection, tag_to_object) = &detections[0];
            let tag_to_cam = self.locate_tag(detection, tag_to_object.0.scaling())?;
            let tag_to_object_iso = na::Isometry3::new(
                tag_to_object.0.isometry.translation.vector,
                tag_to_object.0.isometry.rotation.scaled_axis(),
            );
            return Ok(tag_to_cam * tag_to_object_iso.inverse());
        }

        // More than 1 tag is present. Use `solve_pnp` in OpenCV.
        let mut object_points_data = Vec::<f64>::with_capacity(detections.len() * 12); // `detections.len()` (tags) * `4` (vertices / tag) * `3` (coordinates / vertex)
        let mut image_points_data = Vec::<f64>::with_capacity(detections.len() * 8);
        for (detection, tag_location) in detections {
            for (i, corner) in detection.corners().iter().enumerate() {
                let object_point = tag_location.0.transform_point(&TAG_CORNERS[i]);
                object_points_data.push(object_point.x);
                object_points_data.push(object_point.y);
                object_points_data.push(object_point.z);
                image_points_data.push(corner.x);
                image_points_data.push(corner.y);
            }
        }
        let points_cnt = (detections.len() * 4) as i32;
        let object_points = Mat::new_rows_cols_with_data(points_cnt, 3, &object_points_data)?;
        let image_points = Mat::new_rows_cols_with_data(points_cnt, 2, &image_points_data)?;

        calib3d::solve_pnp(
            &object_points,
            &image_points,
            &self.camera.camera_mat,
            &self.camera.distortion,
            &mut rvec,
            &mut tvec,
            use_extrinsic_guess,
            calib3d::SOLVEPNP_ITERATIVE,
        )?;

        // TODO: invert xyz and rotation here, since solvePnP always returns location on the +z plane.

        let rvec_na = unsafe {
            na::Vector3::new(
                *rvec.at_unchecked::<f64>(0),
                *rvec.at_unchecked::<f64>(1),
                *rvec.at_unchecked::<f64>(2),
            )
        };
        let tvec_na = unsafe {
            na::Vector3::new(
                *tvec.at_unchecked::<f64>(0),
                *tvec.at_unchecked::<f64>(1),
                *tvec.at_unchecked::<f64>(2),
            )
        };

        if let Some(object_index) = object_index {
            // write the rvec and tvec to the object's last location
            self.last_location[object_index] = Some((rvec, tvec, timestamp));
        }

        Ok(na::Isometry3::new(tvec_na, rvec_na))
    }

    /// Locate every object registered in this tagged object locator, then store the results in a
    /// shared mapping from each object's name to their transformation from the camera's frame.
    pub fn locate_objects<'b>(
        &mut self,
        timestamp: SystemTime,
        detections: &'b [apriltag::ApriltagDetection],
        result: Arc<(Mutex<LocatedObjects<'a>>, Condvar)>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Classify each tag into their respective object
        let mut tag_classification: BTreeMap<
            usize,
            Vec<(&'b apriltag::ApriltagDetection, TagLocation)>,
        > = BTreeMap::new();
        for detection in detections {
            let tag_index = TagIndex::new(detection.family()?, detection.id());
            if let Some((registry_index, location)) = self.tag_map.get(&tag_index) {
                tag_classification
                    .entry(*registry_index)
                    .or_default()
                    .push((detection, location.clone()));
            }
        }

        // Lock the result dictionary and write the location results
        let mut locked_result = result.0.lock().unwrap();
        locked_result.timestamp = timestamp;
        locked_result.name_map.clear();
        for (registry_index, detections) in tag_classification {
            let name = self.registry[registry_index].name.as_str();
            locked_result.name_map.insert(
                name,
                self.locate_single_object(&detections, Some(registry_index), timestamp)?,
            );
        }
        drop(locked_result);
        // signal all other threads waiting on this conditional variable
        result.1.notify_all();
        Ok(())
    }

    /// Calculate the Jacobian matrix of the projection mapping.
    ///
    /// The projection function takes in the isometry of the located object as a vector of 3 components:
    /// `x`, `y`, and `z`. It returns a vector of 8n components, where `n` is the number of detected tags
    /// belonging to this object. The returned vector is made of `n` groups of 8 elements, in the order of
    /// `u1`, `v1`, `u2`, `v2`, `u3`, `v3`, `u4`, and `v4`, where each pair of `u` and `v` is the coordinate
    /// of a corner on the tag.
    ///
    /// If we denote the projection map as $f$, then:
    ///
    /// $$
    /// \begin{bmatrix}u_{11} \\ v_{11} \\ u_{12} \\ v_{12} \\ \vdots \\ u_{n4} \\ v_{n4}\end{bmatrix} =
    /// f\left( \begin{bmatrix}x \\ y \\ z \end{bmatrix} \right)
    /// $$
    ///
    /// The Jacobian matrix is an 8nx4 matrix. $J_{ij}$ is the partial derivative of the $i$th component in
    /// the output vector with respect to the $j$th component in the input vector.
    ///
    /// To calculate the projection matrix, a list of detected tags and the computed object location is
    /// required. The tag list has the same format as function `locate_single_object`.
    pub(crate) fn calculate_projection_jacobian<D: Iterator<Item = TagLocation> + Clone>(
        camera_mat: na::Matrix3<f64>,
        detections: D,
        location: na::Isometry3<f64>,
    ) -> Result<na::MatrixXx6<f64>, Box<dyn std::error::Error>> {
        let n = detections.clone().count();
        let mut ans = na::MatrixXx6::<f64>::zeros(8 * n);
        for (i, tag_loc) in detections.enumerate() {
            let rotation = tag_loc.0.isometry.rotation;
            for (j, corner) in TAG_CORNERS.iter().take(4).enumerate() {
                let index = i * 4 + j;
                let u_index = index * 2;
                let local_position = tag_loc.0.transform_point(&corner);
                let a = camera_mat * location.transform_point(&local_position);
                let d_mat = na::Matrix2x3::<f64>::new(
                    1.0 / a.z,
                    0.0,
                    -a.x / (a.z * a.z),
                    0.0,
                    1.0 / a.z,
                    -a.y / (a.z * a.z),
                );
                let translation_jacobian = d_mat * camera_mat;
                ans.fixed_view_mut::<2, 3>(u_index, 0)
                    .copy_from(&translation_jacobian);
                let local_rotation_jacobian = rotation_jacobian(&rotation, &local_position.coords);
                ans.fixed_view_mut::<2, 3>(u_index, 3)
                    .copy_from(&(translation_jacobian * local_rotation_jacobian));
            }
        }
        Ok(ans)
    }

    /// Calculate the covariance matrix of the detection result.
    ///
    /// This function returns a symmetric 3x3 matrix $C_{ij}$, where $C_{xx}$ is the variance (or
    /// in simpler words, uncertainty) on the x direction, $C_{xy}$ is the covariance (or in simpler
    /// words, correlation between uncertainties) on x and y direction, etc.
    ///
    /// The function takes in a pair of numbers `detection_variance`, representing the x and y variance of
    /// each detected corner. This function assumes that each corner's measured coordinate is independent
    /// and identically distributed, with no correlation between the x and y components.
    pub fn calculate_covariance<D: Iterator<Item = TagLocation> + Clone>(
        camera_mat: na::Matrix3<f64>,
        detections: D,
        location: na::Isometry3<f64>,
        detection_variance: (f64, f64),
    ) -> Result<na::Matrix6<f64>, Box<dyn std::error::Error>> {
        let jacobian = Self::calculate_projection_jacobian(camera_mat, detections, location)?;
        let iter = [detection_variance.0, detection_variance.1]
            .into_iter()
            .cycle();
        let mut y = na::DMatrix::zeros(jacobian.nrows(), jacobian.nrows());
        y.set_partial_diagonal(iter); // fills the diagonal of Y matrix with [vx, vy, vx, vy, ...]
        let a = jacobian.transpose() * y * jacobian.clone();
        let b = (jacobian.transpose() * jacobian)
            .try_inverse()
            .ok_or("(J^T * J) does not have an inverse matrix!")?;
        Ok(b * a * b)
        // TODO: this function is not tested. Write a test for this function.
    }
}

#[cfg(test)]
mod tests;
