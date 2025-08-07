use std::error::Error;
use std::fs::File;
use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::{Condvar, Mutex, RwLock};
use std::time::SystemTime;
use std::{collections::HashMap, sync::Arc};
use std::{env, thread};

use map_macro::hash_map;
use opencv::prelude::*;
use opencv::videoio;

use xDIMScreen_locator::camera::{CameraProperty, camera_thread_main};
use xDIMScreen_locator::net::server_thread_main;
use xDIMScreen_locator::tag::apriltag::{ApriltagDetector, ApriltagFamily, ApriltagFamilyType};
use xDIMScreen_locator::tag::locator::{LocatedObjects, TaggedObjectLocator};
use xDIMScreen_locator::tag::locator_thread_main;
use xDIMScreen_locator::tag::tagged_object::{TagIndex, TaggedObject};

fn load_object_from_resources(
    file_name: &'static str,
    id_map: HashMap<String, TagIndex>,
) -> Result<TaggedObject, Box<dyn std::error::Error>> {
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources")
        .join("tagobj")
        .join(file_name);
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let ret = TaggedObject::new_from_json("handheld screen", &tagobj_json, &id_map)?;
    log::info!("Successfully loaded tagobj file {}", tagobj_file_path);
    Ok(ret)
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .try_init()?;

    // prepare camera
    let camera_prop = CameraProperty::new((1680, 945), (None, Some(f64::to_radians(50.0))), None)?;
    let cam_index = 0;
    let mut cam = videoio::VideoCapture::new(cam_index, videoio::CAP_ANY)?;
    cam.set(
        videoio::CAP_PROP_FRAME_WIDTH,
        camera_prop.resolution.0 as f64,
    )?;
    cam.set(
        videoio::CAP_PROP_FRAME_HEIGHT,
        camera_prop.resolution.1 as f64,
    )?;

    // apriltag detector
    let mut family = ApriltagFamilyType::new(ApriltagFamily::Tag36h11);
    let mut detector = ApriltagDetector::new_multithreading(4);
    detector.add_family(&mut family);

    // load objects
    let mut locator = TaggedObjectLocator::new(camera_prop);
    let handheld_screen = load_object_from_resources(
        "handheld-screen.tagobj",
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
        hash_map! {
            "U".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 120),
            "R".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 121),
            "B".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 122),
            "L".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 123),
            "F".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 124),
        },
    )?;
    locator.add(&wand)?;

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
        let locator_thread = s.spawn(move || {
            locator_thread_main(
                termination_signal_clone,
                shared_frame_clone,
                detector,
                locator,
                located_objects,
            )
            .unwrap();
        });

        // start camera thread
        camera_thread_main(
            termination_signal,
            cam,
            shared_frame,
            vec![locator_thread.thread()],
        )
        .unwrap();
    });

    Ok(())
}
