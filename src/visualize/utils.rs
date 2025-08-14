use std::hash::{DefaultHasher, Hash, Hasher};

use plotters::style::RGBColor;

pub(super) fn generate_random_color<H: Hash>(object: H) -> RGBColor {
    let mut hasher = DefaultHasher::new();
    object.hash(&mut hasher);
    let hash = hasher.finish();
    RGBColor(hash as u8, (hash >> 8) as u8, (hash >> 16) as u8)
}
