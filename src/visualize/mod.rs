use std::{
    collections::HashMap,
    sync::{Arc, Condvar, Mutex},
};

use crate::tag::tagged_object::{TagIndex, TagLocation};
use crate::{tag::locator, visualize::chart::VisualizeChart};

pub mod chart;
mod utils;

pub fn visualize_thread_main<'a>(
    object_map: HashMap<String, Vec<(TagIndex, TagLocation)>>,
    located_objects: Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>,
) -> Result<(), Box<dyn std::error::Error>> {
    let native_options = eframe::NativeOptions::default(); // rendering on your desktop
    eframe::run_native(
        "Object Visualizer",
        native_options,
        Box::new(|cc| {
            Ok(Box::new(VisualizeChart::new(
                cc,
                object_map,
                located_objects,
                30.0,
            )))
        }),
    )?;
    Ok(())
}
