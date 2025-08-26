use std::error::Error;
use std::fs::File;
use std::ops::{Range, RangeInclusive};
use std::path::Path;
use std::time::Duration;
use std::{env, thread};

use egui::{CentralPanel, Visuals};
use egui_plotter::{Chart, MouseConfig};
use map_macro::hash_map;

extern crate nalgebra as na;

use plotters::chart::ChartBuilder;
use plotters::prelude::*;
use plotters::series::LineSeries;
use xDIMScreen_locator::tag::apriltag::*;
use xDIMScreen_locator::tag::locator::TAG_CORNERS;
use xDIMScreen_locator::tag::tagged_object::{TagIndex, TaggedObject};

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .try_init()?;

    // load tagged object
    let tagobj_file = Path::new(&env::current_dir()?)
        .join("resources")
        .join("tagobj")
        .join("fractal-tag.tagobj");
    let tagobj_file_path = tagobj_file.to_str().unwrap().to_string();
    let tagobj_json: serde_json::Value = serde_json::from_reader(File::open(tagobj_file)?)?;
    let id_mapping = hash_map! {
        "U".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 120),
        "R".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 121),
        "B".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 122),
        "L".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 123),
        "F".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 124),
        "UL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 0),
        "UR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 1),
        "DL".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 2),
        "DR".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 3),
        "U".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 4),   // TODO: repeated U
        "D".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 5),
        "0".to_string() => TagIndex::new(ApriltagFamily::Tag36h11, 10),
        "1".to_string() => TagIndex::new(ApriltagFamily::Tag25h9, 0),
        "2".to_string() => TagIndex::new(ApriltagFamily::Tag25h9, 1),
        "3".to_string() => TagIndex::new(ApriltagFamily::Tag25h9, 2),
        "4".to_string() => TagIndex::new(ApriltagFamily::Tag25h9, 3),
        "5".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 0),
        "6".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 1),
        "7".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 2),
        "8".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 3),
        "9".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 4),
        "10".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 5),
        "11".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 6),
        "12".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 7),
        "13".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 8),
        "14".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 9),
        "15".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 10),
        "16".to_string() => TagIndex::new(ApriltagFamily::Tag16h5, 11),
    };
    let tagobj = TaggedObject::new_from_json("handheld-screen-v2", &tagobj_json, &id_mapping)?;
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
