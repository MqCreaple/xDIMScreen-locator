use opencv::prelude::*;
use std::ffi::CStr;
use std::fmt::{Debug, Display};
use std::marker::PhantomData;

extern crate nalgebra as na;

use crate::tag::apriltag::apriltag_binding::*;

pub mod apriltag_binding {
    include!(concat!(env!("OUT_DIR"), "/apriltag-bindings.rs"));
}

//-- Apriltag families --//
/// Enumerates all tag families
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApriltagFamily {
    Tag16h5,
    Tag25h9,
    Tag36h10,
    Tag36h11,
    TagCircle21h7,
    TagCircle49h12,
    TagCustom48h12,
    TagStandard41h12,
    TagStandard52h13,
}

impl Display for ApriltagFamily {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl TryFrom<&str> for ApriltagFamily {
    type Error = UnsupportedTagFamilyError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        match value {
            "tag16h5" => Ok(Self::Tag16h5),
            "tag25h9" => Ok(Self::Tag25h9),
            "tag36h10" => Ok(Self::Tag36h10),
            "tag36h11" => Ok(Self::Tag36h11),
            "tagCircle21h7" => Ok(Self::TagCircle21h7),
            "tagCircle49h12" => Ok(Self::TagCircle49h12),
            "tagCustom48h12" => Ok(Self::TagCustom48h12),
            "tagStandard41h12" => Ok(Self::TagStandard41h12),
            "tagStandard52h13" => Ok(Self::TagStandard52h13),
            _ => Err(UnsupportedTagFamilyError {
                name: value.to_string(),
            }),
        }
    }
}

/// A wrapper of the C type `apriltag_family_t` that supports memory allocation and deallocation
pub struct ApriltagFamilyType {
    pub c_type: *mut apriltag_binding::apriltag_family_t,
    family: ApriltagFamily,
}

impl ApriltagFamilyType {
    pub fn new(family: ApriltagFamily) -> Self {
        let c_type = unsafe {
            match family {
                ApriltagFamily::Tag16h5 => tag16h5_create(),
                ApriltagFamily::Tag25h9 => tag25h9_create(),
                ApriltagFamily::Tag36h10 => tag36h10_create(),
                ApriltagFamily::Tag36h11 => tag36h11_create(),
                ApriltagFamily::TagCircle21h7 => tagCircle21h7_create(),
                ApriltagFamily::TagCircle49h12 => tagCircle49h12_create(),
                ApriltagFamily::TagCustom48h12 => tagCustom48h12_create(),
                ApriltagFamily::TagStandard41h12 => tagStandard41h12_create(),
                ApriltagFamily::TagStandard52h13 => tagStandard52h13_create(),
            }
        };
        Self { family, c_type }
    }
}

impl Drop for ApriltagFamilyType {
    fn drop(&mut self) {
        unsafe {
            match self.family {
                ApriltagFamily::Tag16h5 => tag16h5_destroy(self.c_type),
                ApriltagFamily::Tag25h9 => tag25h9_destroy(self.c_type),
                ApriltagFamily::Tag36h10 => tag36h10_destroy(self.c_type),
                ApriltagFamily::Tag36h11 => tag36h11_destroy(self.c_type),
                ApriltagFamily::TagCircle21h7 => tagCircle21h7_destroy(self.c_type),
                ApriltagFamily::TagCircle49h12 => tagCircle21h7_destroy(self.c_type),
                ApriltagFamily::TagCustom48h12 => tagCustom48h12_destroy(self.c_type),
                ApriltagFamily::TagStandard41h12 => tagStandard41h12_destroy(self.c_type),
                ApriltagFamily::TagStandard52h13 => tagStandard52h13_destroy(self.c_type),
            }
        }
    }
}

/// Error type representing an unsupported tag family
pub struct UnsupportedTagFamilyError {
    name: String,
}

impl UnsupportedTagFamilyError {
    pub fn new(name: String) -> Self {
        Self { name }
    }
}

impl Debug for UnsupportedTagFamilyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Tag family \"{}\" is not supported by the current program!",
            self.name
        )
    }
}

impl Display for UnsupportedTagFamilyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl std::error::Error for UnsupportedTagFamilyError {}

//-- Apriltag detector --//

/// Wrapper type of `apriltag_detection` in the apriltag C library.
///
/// This struct is for storing the information of a single apriltag detection on an image
pub struct ApriltagDetection(*mut apriltag_detection);

impl ApriltagDetection {
    pub unsafe fn new_from_raw(raw: *mut apriltag_detection) -> Self {
        Self(raw)
    }

    pub fn id(&self) -> i32 {
        unsafe { (*self.0).id as i32 }
    }

    pub fn family(&self) -> Result<ApriltagFamily, Box<dyn std::error::Error>> {
        let name = unsafe { CStr::from_ptr((*(*self.0).family).name) }.to_str()?;
        let family = ApriltagFamily::try_from(name)?;
        Ok(family)
    }

    pub fn hamming(&self) -> i32 {
        unsafe { (*self.0).hamming as i32 }
    }

    pub fn decision_margin(&self) -> f32 {
        unsafe { (*self.0).decision_margin }
    }

    pub fn homography(&self) -> na::Matrix3<f64> {
        todo!("Conversion from apriltag::Matd to nalgebra::Matrix3 is not yet implemented")
    }

    pub fn center(&self) -> na::Vector2<f64> {
        unsafe { (*self.0).c.into() }
    }

    pub fn corners(&self) -> [na::Vector2<f64>; 4] {
        std::array::from_fn(|i| unsafe { (*self.0).p[i].into() })
    }
}

impl Drop for ApriltagDetection {
    fn drop(&mut self) {
        unsafe {
            apriltag_detection_destroy(self.0);
        }
    }
}

/// Wrapper type of `apriltag_detector` in the apriltag C library.
///
/// The lifetime `'a` is requied such that the tag detector lives shorter than the tag family
/// objects added to the detector.
pub struct ApriltagDetector(*mut apriltag_detector);

impl ApriltagDetector {
    pub fn new() -> Self {
        unsafe { Self(apriltag_detector_create()) }
    }

    pub fn add_family(&mut self, tag_family: &mut ApriltagFamilyType) {
        unsafe { apriltag_detector_add_family_bits(self.0, tag_family.c_type, 2) }
    }

    pub fn remove_family(&mut self, tag_family: &mut ApriltagFamilyType) {
        unsafe { apriltag_detector_remove_family(self.0, tag_family.c_type) }
    }

    pub fn clear_families(&mut self) {
        unsafe {
            apriltag_detector_clear_families(self.0);
        }
    }

    pub fn detect(&mut self, img: &mut image_u8) -> Vec<ApriltagDetection> {
        let z_array = unsafe { apriltag_detector_detect(self.0, img) };
        let z_array_size = unsafe { (*z_array).size as usize };
        let ret = (0..z_array_size)
            .map(|i| unsafe { *((*z_array).data as *const *mut apriltag_detection).add(i) })
            .map(|apriltag_detection_ptr| unsafe {
                ApriltagDetection::new_from_raw(apriltag_detection_ptr)
            })
            .collect::<Vec<_>>();

        // Call `zarray_destroy` to destroy `z_array`.
        //
        // Since the original function is a static inline function, it has to manually written here.
        unsafe {
            if (*z_array).data != std::ptr::null_mut() {
                libc::free((*z_array).data as *mut libc::c_void);
            }
            libc::memset(
                z_array as *mut libc::c_void,
                0,
                std::mem::size_of::<zarray_t>(),
            );
            libc::free(z_array as *mut libc::c_void);
        }

        ret
    }
}

impl Drop for ApriltagDetector {
    fn drop(&mut self) {
        unsafe {
            apriltag_detector_destroy(self.0);
        }
    }
}

//-- Image types --//

pub struct ImageConversionError {
    from_mat: Mat,
}

impl ImageConversionError {
    pub fn new(mat: Mat) -> Self {
        Self { from_mat: mat }
    }
}

impl Debug for ImageConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Unable to convert matrix -\n{:?}\ninto apriltag image format!",
            self.from_mat
        )
    }
}

impl Display for ImageConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Unable to convert matrix -\n{:?}\ninto apriltag image format!",
            self.from_mat
        )
    }
}

impl std::error::Error for ImageConversionError {}

// TODO: Implement these wrapper types with macros after macro_metavar_expr_concat stablizes.

/// Wrapper type of `image_u8` in apriltag C library
pub struct ImageU8(image_u8);

impl ImageU8 {
    pub fn new(width: usize, height: usize) -> Self {
        unsafe {
            Self::new_from_raw(image_u8_create(
                width.try_into().unwrap(),
                height.try_into().unwrap(),
            ))
        }
    }

    pub unsafe fn new_from_raw(raw_img: *mut image_u8) -> Self {
        unsafe { Self(*raw_img) }
    }

    pub fn inner_ref(&self) -> &image_u8 {
        &self.0
    }

    pub fn inner_mut(&mut self) -> &mut image_u8 {
        &mut self.0
    }

    pub fn darken(&mut self) {
        unsafe { image_u8_darken(&mut self.0) }
    }

    pub fn gaussian_blur(&mut self, sigma: f64, k: i32) {
        unsafe {
            image_u8_gaussian_blur(&mut self.0, sigma, k.into());
        }
    }

    pub fn draw_line(&mut self, x0: f32, y0: f32, x1: f32, y1: f32, value: i32, width: i32) {
        unsafe { image_u8_draw_line(&mut self.0, x0, y0, x1, y1, value.into(), width.into()) }
    }

    pub fn draw_circle(&mut self, x0: f32, y0: f32, radius: f32, value: i32) {
        unsafe { image_u8_draw_circle(&mut self.0, x0, y0, radius, value) }
    }
}

impl Clone for ImageU8 {
    fn clone(&self) -> Self {
        unsafe { Self(*image_u8_copy(&self.0)) }
    }
}

impl Drop for ImageU8 {
    fn drop(&mut self) {
        unsafe { image_u8_destroy(&mut self.0) }
    }
}

/// An wrapper type of `image_u8` that does not hold ownership to its internal data.
///
/// This is useful for creating an image from another 2D or 3D data types. For example, if you want
/// to create an image from [opencv::prelude::Mat], the returned image type will be `ImageViewU8<'a, Mat>`.
///
/// This type has internal mutability, meaning that those with access to this type can mutate the content
/// of the image.
///
/// This type does not implement the `Drop` trait. Instead, it needs the parent type to handle memory
/// deallocation.
pub struct ImageU8View<'a, T: 'a> {
    img: image_u8,
    parent: &'a mut T,
}

impl<'a, T: 'a> ImageU8View<'a, T> {
    pub fn inner_ref(&self) -> &image_u8 {
        &self.img
    }

    pub fn inner_mut(&mut self) -> &mut image_u8 {
        &mut self.img
    }

    pub fn darken(&mut self) {
        unsafe { image_u8_darken(&mut self.img) }
    }

    pub fn gaussian_blur(&mut self, sigma: f64, k: i32) {
        unsafe {
            image_u8_gaussian_blur(&mut self.img, sigma, k.into());
        }
    }

    pub fn draw_line(&mut self, x0: f32, y0: f32, x1: f32, y1: f32, value: i32, width: i32) {
        unsafe { image_u8_draw_line(&mut self.img, x0, y0, x1, y1, value.into(), width.into()) }
    }

    pub fn draw_circle(&mut self, x0: f32, y0: f32, radius: f32, value: i32) {
        unsafe { image_u8_draw_circle(&mut self.img, x0, y0, radius, value) }
    }
}

impl<'a> From<&'a mut Mat> for ImageU8View<'a, Mat> {
    fn from(value: &'a mut Mat) -> Self {
        let img_inner = image_u8 {
            width: value.cols(),
            height: value.rows(),
            stride: value.cols(),
            buf: value.data_mut(),
        };
        Self {
            img: img_inner,
            parent: value,
        }
    }
}
