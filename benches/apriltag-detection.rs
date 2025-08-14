use std::env;
use std::hint::black_box;
use std::path::Path;

use criterion::{Criterion, criterion_group, criterion_main};
use opencv::imgcodecs;
use opencv::imgcodecs::IMREAD_GRAYSCALE;
use opencv::prelude::*;
use xDIMScreen_locator::tag::apriltag::{self, ImageU8View};

fn load_image_from_resource(file_name: &str) -> Result<Mat, Box<dyn std::error::Error>> {
    let path = Path::new(&env::current_dir()?)
        .join("resources")
        .join("benchmark")
        .join(file_name);
    let path_name = path.to_str().ok_or(format!(
        "Failed to create path from file name {}",
        file_name
    ))?;
    let image = imgcodecs::imread(path_name, IMREAD_GRAYSCALE)?;
    Ok(image)
}

fn benchmark_apriltag_detection(c: &mut Criterion) {
    let mut tag_family = apriltag::ApriltagFamilyType::new(apriltag::ApriltagFamily::Tag36h11);
    let detector = apriltag::ApriltagDetector::new_multithreading(4).add_family(&mut tag_family);

    for suffix in ["360x225", "720x450", "1440x900", "2880x1800"] {
        let mut image =
            load_image_from_resource(&format!("apriltag-screen-{}.png", suffix)).unwrap();
        let mut image_april = ImageU8View::from(&mut image);

        let bench_name = format!("apriltag detection screen {}", suffix);
        c.bench_function(&bench_name, |b| {
            b.iter(|| black_box(detector.detect(image_april.inner_mut())));
        });
    }
}

criterion_group!(benches, benchmark_apriltag_detection);
criterion_main!(benches);
