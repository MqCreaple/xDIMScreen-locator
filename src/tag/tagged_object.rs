use std::fmt::Display;
use std::ops::RangeInclusive;
use std::{collections::HashMap, hash::Hash};

extern crate nalgebra as na;

use crate::tag::apriltag;
use crate::tag::error::*;

/// Tag index.
///
/// Records the tag's tag type/family and its integer ID.
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub struct TagIndex {
    pub family: apriltag::ApriltagFamily,
    pub id: i32,
}

impl TagIndex {
    pub fn new(family: apriltag::ApriltagFamily, id: i32) -> Self {
        Self { family, id }
    }
}

impl Hash for TagIndex {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u8(self.family as u8);
        state.write_i32(self.id);
    }
}

impl Display for TagIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} ({})", self.id, self.family)
    }
}

/// Tag location
///
/// Records the transformation and scaling relative to the camera's origin.
///
/// For a point [x y z] in the tag's reference frame (e.g. [-1 -1 0] = upleft corner), after
/// applying the transformation, it will be transformed to the camera's reference frame. Note
/// that the `size` variable represents the side length of the tag in real life, it is divided
/// by 2 when initialized.
///
/// You can choose the unit of measurement arbitrarily, but please ensure that all your units
/// are consistent. For example, if you are using milimeter as the unit of length here, please
/// use milimeter everywhere, or the result would be inaccurate.
#[derive(Debug, Clone)]
pub struct TagLocation(pub na::SimilarityMatrix3<f64>);

impl TagLocation {
    pub fn new_size(size: f64) -> Self {
        Self(na::SimilarityMatrix3::new(
            na::Vector3::default(),
            na::Vector3::default(),
            size * 0.5,
        ))
    }

    /// Create a tag location from scaling factor, rotation vector, and translation vector.
    ///
    /// The final transformation is equivalent to first scaling, then rotating, finally
    /// translating the tag by the given amount.
    pub fn new(size: f64, rv: na::Vector3<f64>, tv: na::Vector3<f64>) -> Self {
        Self(na::SimilarityMatrix3::new(tv, rv, size * 0.5))
    }

    pub fn new_from_matrix(size: f64, rm: na::Matrix3<f64>, tv: na::Vector3<f64>) -> Self {
        Self(na::SimilarityMatrix3::from_parts(
            tv.into(),
            na::Rotation3::from_matrix_unchecked(rm),
            size * 0.5,
        ))
    }
}

#[derive(Debug, Clone)]
pub struct TaggedObject {
    pub name: String,
    pub tags: HashMap<TagIndex, TagLocation>,
}

impl TaggedObject {
    /// Create an empty TaggedObject
    pub fn new<S: Into<String>>(name: S) -> Self {
        Self {
            name: name.into(),
            tags: HashMap::new(),
        }
    }

    /// Create a TaggedObject with only one tag and no relative transformation.
    pub fn new_simple<S: Into<String>>(
        name: S,
        family: apriltag::ApriltagFamily,
        id: i32,
        size: f64,
    ) -> Self {
        let mut tags = HashMap::new();
        tags.insert(TagIndex::new(family, id), TagLocation::new_size(size));
        Self {
            name: name.into(),
            tags,
        }
    }

    /// Create a TaggedObject from a tagobj file.
    ///
    /// `tagobj` is the loaded tagobj file (in JSON format). `id_mapping` defines the specific tag family
    /// and tag ID for each template tag in the tagobj file, as the tagobj format doesn't specify each tag's
    /// specific information in it.
    pub fn new_from_json<S: Into<String> + Clone>(
        name: S,
        tagobj: &serde_json::Value,
        id_mapping: &HashMap<String, TagIndex>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        const SUPPORTED_VERSIONS: RangeInclusive<i64> = 1..=1;

        log::info!(
            "Loading tagged object \"{}\" from JSON...",
            name.clone().into()
        );
        let tagobj_object = tagobj.as_object().ok_or(InvalidFormatError::new(
            &tagobj,
            "tagobj file must be an object!",
        ))?;

        let version = tagobj_object
            .get("version")
            .ok_or(InvalidFormatError::new(
                &tagobj,
                "tagobj file must have a \'version\' field!",
            ))?
            .as_i64()
            .ok_or(InvalidFormatError::new(
                &tagobj,
                "tagobj version must be an integer!",
            ))?;
        match version {
            1 => {
                // Version 1 TagObj file
                let tags_json = tagobj_object
                    .get("tags")
                    .ok_or(InvalidFormatError::new(
                        &tagobj,
                        "version 1 tagobj file must have a \'tags\' field!",
                    ))?
                    .as_object()
                    .ok_or(InvalidFormatError::new(
                        &tagobj,
                        "version 1 tagobj file's \'tags\' field must be an array!",
                    ))?;

                let tags = tags_json.iter()
                    .filter_map(|(id_ref, json_value)| {
                        // filter out all invalid entries
                        let id = *id_mapping.get(id_ref)
                            .or_else(|| {
                                log::info!("ID reference \"{}\" in object \"{}\" does not exist in tag ID mapping.", id_ref, name.clone().into());
                                None
                            })?;
                        let json_value = json_value.as_object()
                            .or_else(|| {
                                log::warn!("Invalid format encountered in ID reference \"{}\" in object \"{}\". Skipping.", id_ref, name.clone().into());
                                log::warn!("Entry must be an object type!");
                                None
                            })?;
                        let size = json_value.get("size")
                            .or_else(|| {
                                log::warn!("ID reference \"{}\" in object \"{}\" does not have a \"size\" field. Skipping.", id_ref, name.clone().into());
                                None
                            })?
                            .as_f64()
                            .or_else(|| {
                                log::warn!("The \"size\" field is not a valid floating point number in ID reference \"{}\" in object\"{}\". Skipping.", id_ref, name.clone().into());
                                None
                            })?;
                        // get translation vector
                        let mut tv = json_value.get("tv")
                            .or_else(|| {
                                log::warn!("ID reference \"{}\" in object \"{}\" does not have a \"tv\" field. Skipping.", id_ref, name.clone().into());
                                None
                            })?
                            .as_array()
                            .or_else(|| {
                                log::warn!("Field \"tv\" in ID reference \"{}\" in object \"{}\" is not an array. Skipping.", id_ref, name.clone().into());
                                None
                            })?
                            .iter()
                            .map(|v| v.as_f64().unwrap());
                        let tv = na::Vector3::new(tv.next()?, tv.next()?, tv.next()?);
                        match (json_value.get("rm"), json_value.get("rv")) {
                            (Some(_), Some(_)) => {
                                // Both rotation matrix and rotation vector are defined
                                // Use rotation matrix
                                log::warn!("Both rotation matrix and rotation vector are defined for ID reference \"{}\" in object \"{}\".", id_ref, name.clone().into());
                                log::warn!("Please choose either one to use as the rotation component.");
                                None
                            },
                            (Some(rm), None) => {
                                let rm = rm.as_object()?;
                                let mut rx = rm.get("x")?
                                    .as_array()?
                                    .iter().map(|v| v.as_f64().unwrap());
                                let mut ry = rm.get("y")?
                                    .as_array()?
                                    .iter().map(|v| v.as_f64().unwrap());
                                let mut rz = rm.get("z")?
                                    .as_array()?
                                    .iter().map(|v| v.as_f64().unwrap());
                                let rm = na::Matrix3::new(
                                    rx.next()?, ry.next()?, rz.next()?,
                                    rx.next()?, ry.next()?, rz.next()?,
                                    rx.next()?, ry.next()?, rz.next()?,
                                );
                                Some((id, TagLocation::new_from_matrix(size, rm, tv)))
                            },
                            (None, Some(rv)) => {
                                let mut rv = rv.as_array()?
                                    .iter()
                                    .map(|v| v.as_f64().unwrap());
                                let rv = na::Vector3::new(rv.next()?, rv.next()?, rv.next()?);
                                Some((id, TagLocation::new(size, rv, tv)))
                            },
                            (None, None) => {
                                log::warn!("Neither rotation matrix or rotation vector is defined for ID reference \"{}\" in object \"{}\".", id_ref, name.clone().into());
                                log::warn!("Skipping ID reference {}.", id_ref);
                                None
                            }
                        }
                    })
                    .collect::<HashMap<_, _>>();

                Ok(Self {
                    name: name.into(),
                    tags,
                })
            }
            _ => Err(UnsupportedVersionError::new(version, SUPPORTED_VERSIONS).into()),
        }
    }
}
