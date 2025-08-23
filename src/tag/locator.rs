use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Condvar, Mutex};
use std::time::SystemTime;

use opencv::calib3d;
use opencv::prelude::*;

extern crate nalgebra as na;

use crate::camera::CameraProperty;
use crate::tag::apriltag;
use crate::tag::error::ConflictingTagError;
use crate::tag::tagged_object::{TagIndex, TagLocation, TaggedObject};

pub const TAG_CORNERS: [na::Point3<f64>; 5] = [
    na::Point3::new(-1.0, 1.0, 0.0),
    na::Point3::new(1.0, 1.0, 0.0),
    na::Point3::new(1.0, -1.0, 0.0),
    na::Point3::new(-1.0, -1.0, 0.0),
    na::Point3::new(-1.0, 1.0, 0.0), // add the first point again to make drawing the tag easier
];

pub struct TaggedObjectLocator<'a> {
    /// Camera matrix
    camera: CameraProperty,

    /// List of all objects registered in the object locator
    registry: Vec<&'a TaggedObject>,

    /// Mapping from each tag's property to its corresponding object's index in the registry array
    tag_map: HashMap<TagIndex, (usize, TagLocation)>,
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

    fn locate_single_object<'b, 'c>(
        &self,
        detections: &'b [(&'c apriltag::ApriltagDetection, TagLocation)],
    ) -> Result<na::Isometry3<f64>, Box<dyn std::error::Error>> {
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
            calib3d::SOLVEPNP_ITERATIVE,
        )?;

        let rvec = na::Vector3::new(
            *rvec.at::<f64>(0)?, // TODO: change this to `at_unchecked` when stablizes.
            *rvec.at::<f64>(1)?,
            *rvec.at::<f64>(2)?,
        );
        let tvec = na::Vector3::new(
            *tvec.at::<f64>(0)?, // TODO: change this to `at_unchecked` when stablizes.
            *tvec.at::<f64>(1)?,
            *tvec.at::<f64>(2)?,
        );
        Ok(na::Isometry3::new(tvec, rvec))
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
            locked_result
                .name_map
                .insert(name, self.locate_single_object(&detections)?);
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
    pub(crate) fn calculate_projection_jacobian<'b, 'c>(
        &self,
        detections: &'b [(&'c apriltag::ApriltagDetection, TagLocation)],
        location: na::Isometry3<f64>,
    ) -> Result<na::MatrixXx3<f64>, Box<dyn std::error::Error>> {
        let n = detections.len();
        let camera_mat = self.camera.camera_mat_na()?;
        let mut d_mat = na::MatrixXx3::<f64>::zeros(8 * n);
        for (i, (detection, tag_loc)) in detections.iter().enumerate() {
            for (j, _) in detection.corners().iter().enumerate() {
                let index = i * 4 + j;
                let u_index = index * 2;
                let v_index = index * 2 + 1;
                let local_position = tag_loc.0.transform_point(&TAG_CORNERS[j]);
                let a = camera_mat * location.transform_point(&local_position);
                unsafe {
                    *d_mat.get_unchecked_mut((u_index, 0)) = 1.0 / a.z;
                    *d_mat.get_unchecked_mut((u_index, 2)) = -a.x / (a.z * a.z);
                    *d_mat.get_unchecked_mut((v_index, 1)) = 1.0 / a.z;
                    *d_mat.get_unchecked_mut((v_index, 2)) = -a.y / (a.z * a.z);
                }
            }
        }
        Ok(d_mat * camera_mat)
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
    pub fn calculate_covariance<'b, 'c>(
        &self,
        detections: &'b [(&'c apriltag::ApriltagDetection, TagLocation)],
        location: na::Isometry3<f64>,
        detection_variance: (f64, f64),
    ) -> Result<na::Matrix3<f64>, Box<dyn std::error::Error>> {
        let jacobian = self.calculate_projection_jacobian(detections, location)?;
        let iter = [detection_variance.0, detection_variance.1].into_iter().cycle();
        let mut y = na::DMatrix::zeros(jacobian.nrows(), jacobian.nrows());
        y.set_partial_diagonal(iter);   // fills the diagonal of Y matrix with [vx, vy, vx, vy, ...]
        let a = jacobian.transpose() * y * jacobian.clone();
        let b = (jacobian.transpose() * jacobian).try_inverse().ok_or("(J^T * J) does not have an inverse matrix!")?;
        Ok(b * a * b)
        // TODO: this function is not tested. Write a test for this function.
    }
}

#[cfg(test)]
mod tests {
    extern crate nalgebra as na;

    use std::ops::DerefMut;

    use rand::Rng;

    use crate::tag::apriltag::{
        ApriltagDetection, ApriltagFamily, ApriltagFamilyType, apriltag_binding,
    };

    use super::*;

    #[test]
    fn test_projection_jacobian() {
        let camera =
            CameraProperty::new((1920, 1080), (None, Some(f64::to_radians(50.0))), None).unwrap();
        let camera_mat = camera.camera_mat_na().unwrap();

        let mut object = TaggedObject::new("test object");
        let tag36h11_family = ApriltagFamilyType::new(apriltag::ApriltagFamily::Tag36h11);
        object.tags.insert(
            TagIndex {
                family: ApriltagFamily::Tag36h11,
                id: 0,
            },
            TagLocation::new(
                1.0,
                na::Vector3::default(),
                na::vector![0.0, 0.0, 0.0]
            ),
        );
        object.tags.insert(
            TagIndex {
                family: ApriltagFamily::Tag36h11,
                id: 1,
            },
            TagLocation::new(
                1.0,
                na::Vector3::default(),
                na::vector![0.0, 1.0, 2.0]
            ),
        );
        object.tags.insert(
            TagIndex {
                family: ApriltagFamily::Tag36h11,
                id: 2,
            },
            TagLocation::new(
                1.0,
                na::Vector3::default(),
                na::vector![-2.0, 1.0, 0.0],
            ),
        );
        let mut locator = TaggedObjectLocator::new(camera);
        locator.add(&object).unwrap();

        let mut rng = rand::rng();
        for _ in 0..100 {
            // simulate a projection from a random location
            let object_location = na::Isometry3::new(
                na::vector![rng.random_range(-3.0..3.0), rng.random_range(-3.0..3.0), rng.random_range(4.0..12.0)], // TODO: use random numbers
                na::vector![0.0, 0.05, 0.02],
            );
            let mut detections = Vec::with_capacity(object.tags.len());
            for (index, tag_location) in &object.tags {
                let center = camera_mat
                    * object_location.transform_point(
                        &tag_location
                            .0
                            .transform_point(&na::Point3::new(0.0, 0.0, 0.0)),
                    );
                let corners = std::array::from_fn(|i| {
                    let point = camera_mat
                        * object_location
                            .transform_point(&tag_location.0.transform_point(&TAG_CORNERS[i]));
                    [point.x / point.z, point.y / point.z]
                });
                let dummy_h_matd = unsafe { apriltag_binding::matd_create(2, 2) };
                let mut detection_raw = Box::new(apriltag_binding::apriltag_detection {
                    family: tag36h11_family.c_type,
                    id: index.id,
                    hamming: 0,
                    decision_margin: 0.0,
                    H: dummy_h_matd,
                    c: [center.x / center.z, center.y / center.z],
                    p: corners,
                });
                let detection =
                    unsafe { ApriltagDetection::new_from_raw(detection_raw.deref_mut()) };
                std::mem::forget(detection_raw);
                detections.push((detection, tag_location.clone()));
            }

            // calculate projection jacobian
            let detections2 = detections
                .iter()
                .map(|(detection, location)| (detection, location.clone()))
                .collect::<Vec<_>>();
            let calculated_projection_jacobian = locator
                .calculate_projection_jacobian(&detections2, object_location)
                .unwrap();

            assert_eq!(calculated_projection_jacobian.nrows(), 8 * object.tags.len());

            // measure the projection jacobian with numerical differentiation
            let mut measured_projection_jacobian =
                na::MatrixXx3::zeros(calculated_projection_jacobian.nrows());
            let dx = 1e-4;
            for (i, shift) in [
                na::vector![dx, 0.0, 0.0],
                na::vector![0.0, dx, 0.0],
                na::vector![0.0, 0.0, dx],
            ].into_iter().enumerate() {
                let mut object_location_1 = object_location.clone();
                object_location_1.append_translation_mut(&na::Translation3::from(-shift));
                let mut vec1 = na::DVector::zeros(calculated_projection_jacobian.nrows());
                for (j, (_, tag_location)) in object.tags.iter().enumerate() {
                    for k in 0..4 {
                        let point = camera_mat
                            * object_location_1
                                .transform_point(&tag_location.0.transform_point(&TAG_CORNERS[k]));
                        vec1[j * 8 + k * 2] = point.x / point.z;
                        vec1[j * 8 + k * 2 + 1] = point.y / point.z;
                    }
                }
                let mut object_location_2 = object_location.clone();
                object_location_2.append_translation_mut(&na::Translation3::from(shift));
                let mut vec2 = na::DVector::zeros(calculated_projection_jacobian.nrows());
                for (j, (_, tag_location)) in object.tags.iter().enumerate() {
                    for k in 0..4 {
                        let point = camera_mat
                            * object_location_2
                                .transform_point(&tag_location.0.transform_point(&TAG_CORNERS[k]));
                        vec2[j * 8 + k * 2] = point.x / point.z;
                        vec2[j * 8 + k * 2 + 1] = point.y / point.z;
                    }
                }
                let diff = (vec2 - vec1) / (2.0 * dx);
                measured_projection_jacobian.set_column(i, &diff);
            }

            for i in 0..calculated_projection_jacobian.nrows() {
                for j in 0..3 {
                    let measured_i_j = measured_projection_jacobian.get((i, j)).unwrap();
                    let calculated_i_j = calculated_projection_jacobian.get((i, j)).unwrap();
                    assert!(
                        (measured_i_j - calculated_i_j).abs() <= 0.01 * calculated_i_j.abs(),    // less than 1% of relative error
                        "Assertion failed on index {} {}, with the measured value being {} and calculated value being {}",
                        i, j, measured_i_j, calculated_i_j
                    );
                }
            }
        }
    }
}
