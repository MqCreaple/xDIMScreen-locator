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
}
