use std::collections::HashMap;
use std::fs::File;
use std::path::Path;
use std::env;
use std::error::Error;

use map_macro::hash_map;

use xDIMScreen_locator::tag::apriltag::*;
use xDIMScreen_locator::tag::tagged_object::{TagIndex, TaggedObject};

fn main() -> Result<(), Box<dyn Error>> {
    // load tagged object
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources").join("tagobj").join("handheld-screen.tagobj");
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let id_mapping = hash_map! {
        "UL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 0),
        "UR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 1),
        "BL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 2),
        "BR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 3),
    };
    let tagobj = TaggedObject::new_from_json("handheld screen", &tagobj_json, &id_mapping)?;
    println!("Successfully loaded tagged object from path {}", tagobj_file_path);
    for (id, tag) in tagobj.tags.iter() {
        println!();
        println!("Tag id: {}", id);
        println!("Transformation matrix:");
        println!("{:?}", tag.0);
    }
    // TODO: write tagged object detection
    Ok(())
}