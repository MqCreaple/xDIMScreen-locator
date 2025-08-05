use std::env;
use std::f64;
use std::fs::File;
use std::path::Path;
use std::sync::Condvar;
use std::sync::{Arc, Mutex};

use map_macro::hash_map;
use opencv::prelude::*;
use opencv::videoio;

use xDIMScreen_locator::camera::CameraProperty;
use xDIMScreen_locator::tag::apriltag::{ApriltagDetector, ApriltagFamily, ApriltagFamilyType};
use xDIMScreen_locator::tag::locator::LocatedObjects;
use xDIMScreen_locator::tag::locator_thread_main;
use xDIMScreen_locator::tag::tagged_object::TagIndex;
use xDIMScreen_locator::tag::{locator::TaggedObjectLocator, tagged_object::TaggedObject};

fn load_handheld_screen() -> Result<TaggedObject, Box<dyn std::error::Error>> {
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources")
        .join("tagobj")
        .join("handheld-screen.tagobj");
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let id_mapping = hash_map! {
        "UL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 0),
        "UR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 1),
        "DL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 2),
        "DR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 3),
    };
    let ret = TaggedObject::new_from_json("handheld screen", &tagobj_json, &id_mapping)?;
    log::info!("Successfully loaded tagobj file {}", tagobj_file_path);
    Ok(ret)
}

fn load_wand_object() -> Result<TaggedObject, Box<dyn std::error::Error>> {
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources")
        .join("tagobj")
        .join("wand.tagobj");
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let id_mapping = hash_map! {
        "U".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 120),
        "R".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 121),
        "B".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 122),
        "L".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 123),
        "F".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 124),
    };
    let ret = TaggedObject::new_from_json("wand", &tagobj_json, &id_mapping)?;
    log::info!("Successfully loaded tagobj file {}", tagobj_file_path);
    Ok(ret)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .try_init()?;

    // camera
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
    let mut detector = ApriltagDetector::new();
    detector.add_family(&mut family);

    // object locator
    let mut locator = TaggedObjectLocator::new(camera_prop);
    let handheld_screen = load_handheld_screen()?;
    locator.add(&handheld_screen)?;
    let wand = load_wand_object()?;
    locator.add(&wand)?;
    let located_objects = Arc::new((Mutex::new(LocatedObjects::new()), Condvar::new()));

    // start locator thread
    locator_thread_main(cam, detector, locator, located_objects)?;

    // TODO: Heap corruption when dropping apriltag detector
    Ok(())
}
