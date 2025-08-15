use std::hash::{DefaultHasher, Hash, Hasher};

use plotters::style::RGBColor;

extern crate nalgebra as na;

pub const AXES: [na::Point3<f64>; 3] = [
    na::Point3::new(1.0, 0.0, 0.0),
    na::Point3::new(0.0, 1.0, 0.0),
    na::Point3::new(0.0, 0.0, 1.0),
];
pub const AXES_COLORS: [(u8, u8, u8); 3] = [(255, 0, 0), (0, 255, 0), (0, 0, 255)];

pub(crate) fn generate_random_color<H: Hash>(object: H) -> RGBColor {
    let mut hasher = DefaultHasher::new();
    object.hash(&mut hasher);
    let hash = hasher.finish();
    RGBColor(hash as u8, (hash >> 8) as u8, (hash >> 16) as u8)
}
