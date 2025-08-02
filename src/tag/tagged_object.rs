use std::fmt::Display;
use std::ops::RangeInclusive;
use std::{collections::HashMap, hash::Hash};

use opencv::{calib3d, prelude::*};

extern crate nalgebra as na;

use crate::tag::error::*;
use crate::tag::apriltag;

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
/// Records the transformation and scaling relative to the camera's origin by a 3x4 matrix.
/// 
/// For a point [x y z] in the tag's reference frame ([-1 -1 0] = upleft corner), when
/// appending a 1 to make it homogeneous and then multiply it with the matrix, you will
/// get the point's relative position to the object's center.
/// 
/// You can choose the unit of measurement arbitrarily, but please ensure that all your units
/// are consistent. For example, if you are using milimeter as the unit of length here, please
/// use milimeter everywhere, or the result would be inaccurate.
#[derive(Debug, Clone)]
pub struct TagLocation(pub na::Affine3<f64>);

impl TagLocation {
    pub fn new_scale(scale: f64) -> Self {
        let mut mat = na::Matrix4::<f64>::zeros();
        (0..3).for_each(|i| {
            unsafe { *mat.get_unchecked_mut((i, i)) = scale; }
        });
        unsafe { *mat.get_unchecked_mut((4, 4)) = 1.0; }
        Self(na::Affine3::from_matrix_unchecked(mat))
    }

    /// Create a tag location from scaling factor, rotation vector, and translation vector.
    /// 
    /// The final transformation is equivalent to first scaling, then rotating, finally
    /// translating the tag by the given amount.
    pub fn new_scale_rv_tv(scale: f64, rv: na::Vector3<f64>, tv: na::Vector3<f64>) -> Result<Self, opencv::Error> {
        let mut transformation_mat = na::Matrix4::<f64>::zeros();
        let mut rotate_mat = Mat::default();
        let rotate_vec = Mat::new_rows_cols_with_data(3, 1, rv.as_slice())?;
        let mut _jacobian = Mat::default();
        calib3d::rodrigues(&rotate_vec, &mut rotate_mat, &mut _jacobian)?;
        rotate_mat = (rotate_mat * scale).into_result()?.to_mat()?;    // apply scaling to rotation matrix
        // copy the rotation&scaling matrix to the 3x3 corner of `rot_trans_mat`
        for i in 0..3 {
            for j in 0..3 {
                unsafe {
                    *transformation_mat.get_unchecked_mut((i, j)) = *rotate_mat.at_2d(i as i32, j as i32)?;
                }
            }
        }
        // copy the translation vector to the 4th row of `rot_trans_mat`
        unsafe {
            *transformation_mat.get_unchecked_mut((0, 3)) = tv.x;
            *transformation_mat.get_unchecked_mut((1, 3)) = tv.y;
            *transformation_mat.get_unchecked_mut((2, 3)) = tv.z;
            *transformation_mat.get_unchecked_mut((3, 3)) = 1.0;
        }
        Ok(Self(na::Affine3::from_matrix_unchecked(transformation_mat)))
    }
}

pub struct TaggedObject {
    pub name: String,
    pub tags: HashMap<TagIndex, TagLocation>,
}

impl TaggedObject {
    /// Create an empty TaggedObject
    pub fn new<S: Into<String>>(name: S) -> Self {
        Self { name: name.into(), tags: HashMap::new() }
    }

    /// Create a TaggedObject with only one tag and no relative transformation.
    pub fn new_simple<S: Into<String>>(name: S, family: apriltag::ApriltagFamily, id: i32, scale: f64) -> Self {
        let mut tags = HashMap::new();
        tags.insert(TagIndex::new(family, id), TagLocation::new_scale(scale));
        Self { name: name.into(), tags }
    }

    /// Create a TaggedObject from a tagobj file.
    /// 
    /// `tagobj` is the loaded tagobj file (in JSON format). `id_mapping` defines the specific tag family
    /// and tag ID for each template tag in the tagobj file, as the tagobj format doesn't specify each tag's
    /// specific information in it.
    pub fn new_from_json<S: Into<String>>(name: S, tagobj: &serde_json::Value, id_mapping: &HashMap<String, TagIndex>) -> Result<Self, Box<dyn std::error::Error>> {
        const SUPPORTED_VERSIONS: RangeInclusive<i64> = 1..=1;

        let tagobj_object = tagobj.as_object()
            .ok_or(InvalidFormatError::new(&tagobj, "tagobj file must be an object!"))?;

        let version = tagobj_object.get("version")
            .ok_or(InvalidFormatError::new(&tagobj, "tagobj file must have a \'version\' field!"))?
            .as_i64()
            .ok_or(InvalidFormatError::new(&tagobj, "tagobj version must be an integer!"))?;
        match version {
            1 => {
                // Version 1 TagObj file
                let tags_json = tagobj_object.get("tags")
                    .ok_or(InvalidFormatError::new(&tagobj, "version 1 tagobj file must have a \'tags\' field!"))?
                    .as_object()
                    .ok_or(InvalidFormatError::new(&tagobj, "version 1 tagobj file's \'tags\' field must be an array!"))?;
                let tags = tags_json.iter()
                    .filter_map(|(id_ref, json_value)| {
                        // filter out all invalid entries
                        let id = *id_mapping.get(id_ref)?;
                        let json_value = json_value.as_object()?;
                        let scale = json_value.get("scale")?.as_f64()?;
                        let mut rv = json_value.get("rv")?
                            .as_array()?
                            .iter()
                            .map(|v| v.as_f64().unwrap());
                        let rv = na::Vector3::new(rv.next()?, rv.next()?, rv.next()?);
                        let mut tv = json_value.get("tv")?
                            .as_array()?
                            .iter()
                            .map(|v| v.as_f64().unwrap());
                        let tv = na::Vector3::new(tv.next()?, tv.next()?, tv.next()?);
                        Some((id, TagLocation::new_scale_rv_tv(scale, rv, tv).unwrap()))
                    })
                    .collect::<HashMap<_, _>>();
                Ok(Self { name: name.into(), tags })
            },
            _ => Err(UnsupportedVersionError::new(version, SUPPORTED_VERSIONS).into())
        }
    }
}