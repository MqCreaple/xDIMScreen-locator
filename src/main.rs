use std::error::Error;
use std::fs::File;
use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::{Condvar, Mutex, RwLock};
use std::time::SystemTime;
use std::{collections::HashMap, sync::Arc};
use std::{env, thread};

use clap::Parser;
use map_macro::hash_map;
use opencv::prelude::*;
use opencv::videoio;

use xDIMScreen_locator::camera::{CameraProperty, camera_thread_main};
use xDIMScreen_locator::net::server_thread_main;
use xDIMScreen_locator::tag::apriltag::{ApriltagDetector, ApriltagFamily, ApriltagFamilyType};
use xDIMScreen_locator::tag::locator::{LocatedObjects, TaggedObjectLocator};
use xDIMScreen_locator::tag::locator_thread_main;
use xDIMScreen_locator::tag::tagged_object::{TagIndex, TaggedObject};

#[cfg(feature = "visualize")]
use xDIMScreen_locator::visualize::visualize_thread_main;

fn load_object_from_resources(
    file_name: &'static str,
    object_name: &'static str,
    id_map: HashMap<String, TagIndex>,
) -> Result<TaggedObject, Box<dyn std::error::Error>> {
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources")
        .join("tagobj")
        .join(file_name);
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let ret = TaggedObject::new_from_json(object_name, &tagobj_json, &id_map)?;
    log::info!("Successfully loaded tagobj file {}", tagobj_file_path);
    Ok(ret)
}

#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// The device index of the camera to calibrate. Laptop's builtin camera is usually at index 0.
    #[arg(short, long, default_value_t = 0)]
    cam_id: i32,

    /// The camera resolution's X component.
    #[arg(long, default_value_t = 1920)]
    cam_res_x: u32,

    /// The camera resolution's Y component.
    #[arg(long, default_value_t = 1080)]
    cam_res_y: u32,

    /// The camera's field of view on x direction. Unit: degrees. Not necessary if the camera matrix is provided.
    #[arg(long)]
    cam_fov_x: Option<f64>,

    /// The camera's field of view on x direction. Unit: degrees. Not necessary if the camera matrix is provided.
    #[arg(long)]
    cam_fov_y: Option<f64>,

    /// Number of threads used by the apriltag detector.
    #[arg(long, default_value_t = 4)]
    detector_nthreads: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .try_init()?;
    let args = Args::parse();

    // prepare camera
    let camera_prop = CameraProperty::new(
        (args.cam_res_x, args.cam_res_y),
        (
            args.cam_fov_x.map(f64::to_radians),
            args.cam_fov_y.map(f64::to_radians),
        ),
        None,
    )?;
    log::info!("Camera matrix: {}", camera_prop.camera_mat_na().unwrap());
    let mut cam = videoio::VideoCapture::new(args.cam_id, videoio::CAP_ANY)?;
    cam.set(
        videoio::CAP_PROP_FRAME_WIDTH,
        camera_prop.resolution.0 as f64,
    )?;
    cam.set(
        videoio::CAP_PROP_FRAME_HEIGHT,
        camera_prop.resolution.1 as f64,
    )?;

    // load objects
    let mut locator = TaggedObjectLocator::new(camera_prop.clone());
    let handheld_screen = load_object_from_resources(
        "handheld-screen.tagobj",
        "handheld screen",
        hash_map! {
            "UL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 0),
            "UR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 1),
            "DL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 2),
            "DR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 3),
        },
    )?;
    locator.add(&handheld_screen)?;
    let wand = load_object_from_resources(
        "wand.tagobj",
        "wand",
        hash_map! {
            "U".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 120),
            "R".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 121),
            "B".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 122),
            "L".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 123),
            "F".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 124),
        },
    )?;
    locator.add(&wand)?;
    let fractal_tag = load_object_from_resources(
        "fractal-tag.tagobj",
        "fractal tag",
        hash_map! {
            "0".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 10),
            "1".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 11),
            "2".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 15),
            "3".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 19),
            "4".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 23),
            "5".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 12),
            "6".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 13),
            "7".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 14),
            "8".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 16),
            "9".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 17),
            "10".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 18),
            "11".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 20),
            "12".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 21),
            "13".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 22),
            "14".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 24),
            "15".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 25),
            "16".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 26),
        },
    )?;
    locator.add(&fractal_tag)?;

    // A thread scope is used here to resolve the lifetime issue.
    // Otherwise, the compiler will think that the objects need to be borrowed for 'static.
    thread::scope(|s| {
        let termination_signal = Arc::new(AtomicBool::new(false));
        let shared_frame = Arc::new(RwLock::new((Mat::default(), SystemTime::UNIX_EPOCH)));
        let located_objects = Arc::new((Mutex::new(LocatedObjects::new()), Condvar::new()));

        // start server thread
        let termination_signal_clone = termination_signal.clone();
        let located_objects_clone = located_objects.clone();
        let _ = s.spawn(move || {
            server_thread_main(termination_signal_clone, 30002, located_objects_clone).unwrap()
        });

        // start locator thread
        let termination_signal_clone = termination_signal.clone();
        let shared_frame_clone = shared_frame.clone();
        let located_objects_clone = located_objects.clone();

        #[cfg(feature = "visualize")]
        let object_map = locator.get_object_map(); // this object need to be created before locator thread launches

        let locator_thread = s.spawn(move || {
            // construct the apriltag detector in the locator thread
            let mut family_tag36h11 = ApriltagFamilyType::new(ApriltagFamily::Tag36h11);
            let detector = ApriltagDetector::new_multithreading(args.detector_nthreads)
                .add_family(&mut family_tag36h11)
                .quad_sigma(-10.0);

            locator_thread_main(
                termination_signal_clone,
                shared_frame_clone,
                detector,
                locator,
                located_objects_clone,
            )
            .unwrap();
        });

        // start camera thread
        let _ = s.spawn(move || {
            camera_thread_main(
                termination_signal,
                cam,
                shared_frame,
                vec![locator_thread.thread()],
            )
            .unwrap();
        });

        // start visualize thread
        #[cfg(feature = "visualize")]
        let located_objects_clone = located_objects.clone();
        #[cfg(feature = "visualize")]
        visualize_thread_main(camera_prop, object_map, located_objects_clone).unwrap(); // visualizer must be in the main thread
    });

    Ok(())
}
