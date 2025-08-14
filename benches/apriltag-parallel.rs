use std::hint::black_box;
use std::path::Path;

use criterion::Criterion;
use criterion::criterion_group;
use criterion::criterion_main;
use opencv::imgcodecs;
use opencv::prelude::*;
use xDIMScreen_locator::tag::apriltag::ImageU8View;
use xDIMScreen_locator::tag::apriltag::apriltag_binding;

fn load_image_from_resource(file_name: &str) -> Result<Mat, Box<dyn std::error::Error>> {
    let path = Path::new(&std::env::current_dir()?)
        .join("resources")
        .join("benchmark")
        .join(file_name);
    let path_name = path.to_str().ok_or(format!(
        "Failed to create path from file name {}",
        file_name
    ))?;
    let image = imgcodecs::imread(path_name, imgcodecs::IMREAD_GRAYSCALE)?;
    Ok(image)
}

fn benchmark_gaussian_blur(c: &mut Criterion) {
    const NUM_THREADS: [usize; 2] = [2, 4];
    let workerpools = NUM_THREADS.map(|nthreads| unsafe {
        apriltag_binding::workerpool_create(nthreads as std::os::raw::c_int)
    });

    for suffix in [
        "320x180",
        "640x360",
        "960x540",
        "1024x576",
        "1280x720",
        "1536x864",
        "1920x1080",
    ] {
        let mut bench_group = c.benchmark_group(format!("apriltag gaussian blur {}", suffix));
        bench_group.sample_size(10000);

        // single threaded gaussian blur
        let mut image = load_image_from_resource(&format!("apriltag-real-{}.jpg", suffix)).unwrap();
        let mut image_april = ImageU8View::from(&mut image);

        bench_group.bench_function("single thread", |b| {
            b.iter(|| {
                black_box(unsafe {
                    apriltag_binding::image_u8_gaussian_blur(image_april.inner_mut(), 8.0, 17)
                })
            });
        });

        for i in 0..NUM_THREADS.len() {
            // gaussian blur with n threads
            let mut image_april = ImageU8View::from(&mut image);
            bench_group.bench_function(format!("{} threads", NUM_THREADS[i]), |b| {
                b.iter(|| {
                    black_box(unsafe {
                        apriltag_binding::image_u8_gaussian_blur_parallel(
                            workerpools[i],
                            image_april.inner_mut(),
                            8.0,
                            17,
                        )
                    })
                });
            });
        }

        bench_group.finish();
    }

    for wp in workerpools {
        unsafe { apriltag_binding::workerpool_destroy(wp) };
    }
}

criterion_group!(benches, benchmark_gaussian_blur);
criterion_main!(benches);
