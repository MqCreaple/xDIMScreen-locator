use std::hint::black_box;

use criterion::Criterion;
use criterion::criterion_group;
use criterion::criterion_main;
use opencv::prelude::*;
use opencv::videoio;

fn benchmark_camera_fps(c: &mut Criterion) {
    const CAM_INDEX: i32 = 0;

    for resolution in [(1920, 1080), (1280, 720), (960, 540), (640, 360)] {
        let mut cam = videoio::VideoCapture::new(CAM_INDEX, videoio::CAP_ANY).unwrap();
        cam.set(videoio::CAP_PROP_FRAME_WIDTH, resolution.0 as f64)
            .unwrap();
        cam.set(videoio::CAP_PROP_FRAME_HEIGHT, resolution.1 as f64)
            .unwrap();

        let bench_name = format!("camera FPS {}x{}", resolution.0, resolution.1);
        c.bench_function(&bench_name, |b| {
            b.iter(|| {
                let mut _frame = Mat::default();
                black_box(cam.read(&mut _frame).unwrap());
            });
        });
    }
}

criterion_group!(benches, benchmark_camera_fps);
criterion_main!(benches);
