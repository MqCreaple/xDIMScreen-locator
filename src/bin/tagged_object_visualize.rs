use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::ops::{Range, RangeInclusive};
use std::path::Path;
use std::time::Duration;
use std::{env, thread};

use egui::{CentralPanel, Visuals};
use egui_plotter::{Chart, MouseConfig};

extern crate nalgebra as na;

use clap::Parser;
use plotters::chart::ChartBuilder;
use plotters::prelude::*;
use plotters::series::LineSeries;
use xDIMScreen_locator::tag::apriltag::*;
use xDIMScreen_locator::tag::locator::TAG_CORNERS;
use xDIMScreen_locator::tag::tagged_object::{TagIndex, TaggedObject};

#[derive(Parser, Debug)]
#[command(
    name = "xDIMScreen tagged object visualizer",
    version,
    about = "Visualize tagged object from TagObj files."
)]
struct Args {
    /// Name of the TagObj file to load from. The suffix ".tagobj" is optional.
    #[arg(default_value_t = String::from("simple-tag"))]
    name: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .try_init()?;
    let mut args = Args::parse();
    if !args.name.ends_with(".tagobj") {
        args.name += ".tagobj";
    }

    // load tagged object
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources")
        .join("tagobj")
        .join(args.name.clone());
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let mut id_mapping = HashMap::new();
    let mut next_tag_index = 0;
    for tag_id in tagobj_json.get("tags").unwrap().as_object().unwrap().keys() {
        id_mapping.insert(
            tag_id.clone(),
            TagIndex::new(ApriltagFamily::Tag36h11, next_tag_index),
        );
        next_tag_index += 1;
    }
    let tagobj = TaggedObject::new_from_json(args.name, &tagobj_json, &id_mapping)?;
    println!(
        "Successfully loaded tagged object from path {}",
        tagobj_file_path
    );
    for (id, tag) in tagobj.tags.iter() {
        println!();
        println!("Tag id: {}", id);
        println!("Transformation matrix:");
        println!("{:?}", tag.0);
    }

    // Visualize the object
    let native_options = eframe::NativeOptions::default(); // rendering on your desktop
    eframe::run_native(
        "Demo Project",
        native_options,
        Box::new(|cc| Ok(Box::new(VisualizeChart::new(cc, tagobj.clone())))),
    )?;
    Ok(())
}

struct VisualizeChart {
    chart: Chart<()>,
}

fn to_f64_range(range: &RangeInclusive<i32>) -> Range<f64> {
    range.start().as_f64()..range.end().as_f64()
}

impl VisualizeChart {
    pub fn new(cc: &eframe::CreationContext<'_>, tagobj: TaggedObject) -> Self {
        let context = &cc.egui_ctx;
        context.set_visuals(Visuals::light()); // Set to light theme
        let chart = Chart::new(())
            .mouse(MouseConfig::enabled())
            .pitch(0.7)
            .yaw(0.7)
            .builder_cb(Box::new(move |area, transform, _d| {
                let x_axis = 0..=5;
                let y_axis = 0..=5;
                let z_axis = 0..=5;

                let mut chart = ChartBuilder::on(&area)
                    .build_cartesian_3d(
                        to_f64_range(&x_axis),
                        to_f64_range(&y_axis),
                        to_f64_range(&z_axis),
                    )
                    .unwrap();

                chart.with_projection(|mut pb| {
                    pb.yaw = transform.yaw;
                    pb.pitch = transform.pitch;
                    pb.scale = transform.zoom;
                    pb.into_matrix()
                });

                // chart.configure_axes()
                //     .light_grid_style(BLACK.mix(0.15))
                //     .max_light_lines(3)
                //     .draw().unwrap();

                chart
                    .draw_series(LineSeries::new(x_axis.map(|x| (x.as_f64(), 0., 0.)), &RED))
                    .unwrap()
                    .label("x axis");
                chart
                    .draw_series(LineSeries::new(
                        y_axis.map(|y| (0., y.as_f64(), 0.)),
                        &GREEN,
                    ))
                    .unwrap()
                    .label("y axis");
                chart
                    .draw_series(LineSeries::new(z_axis.map(|z| (0., 0., z.as_f64())), &BLUE))
                    .unwrap()
                    .label("z axis");

                // Draw series here
                for (_, location) in &tagobj.tags {
                    // Extract all the corner points
                    chart
                        .draw_series(LineSeries::new(
                            TAG_CORNERS.iter().map(|point| {
                                let transformed = location.0.transform_point(point);
                                (transformed.x, transformed.y, transformed.z)
                            }),
                            &BLACK,
                        ))
                        .unwrap();
                    chart
                        .draw_series(TAG_CORNERS.iter().take(4).map(|point| {
                            let transformed = location.0.transform_point(point);
                            Circle::new((transformed.x, transformed.y, transformed.z), 6.0, &BLACK)
                        }))
                        .unwrap();

                    // draw the x and y axis of each tag
                    const X_AXIS: [na::Point3<f64>; 2] = [
                        na::Point3::new(0.0, 0.0, 0.0),
                        na::Point3::new(0.2, 0.0, 0.0),
                    ];
                    const Y_AXIS: [na::Point3<f64>; 2] = [
                        na::Point3::new(0.0, 0.0, 0.0),
                        na::Point3::new(0.0, 0.2, 0.0),
                    ];
                    chart
                        .draw_series(LineSeries::new(
                            X_AXIS.iter().map(|point| {
                                let transformed = location.0.transform_point(point);
                                (transformed.x, transformed.y, transformed.z)
                            }),
                            &RED,
                        ))
                        .unwrap();
                    chart
                        .draw_series(LineSeries::new(
                            Y_AXIS.iter().map(|point| {
                                let transformed = location.0.transform_point(point);
                                (transformed.x, transformed.y, transformed.z)
                            }),
                            &GREEN,
                        ))
                        .unwrap();
                }
            }));

        Self { chart }
    }
}

impl eframe::App for VisualizeChart {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        const FPS: f64 = 60.0;

        CentralPanel::default().show(ctx, |ui| {
            self.chart.draw(ui);
        });

        thread::sleep(Duration::from_secs_f64(1.0 / FPS));
        ctx.request_repaint();
    }
}
