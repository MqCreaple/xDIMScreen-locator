use std::collections::HashMap;
use std::marker::PhantomData;
use std::ops::{Range, RangeInclusive};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

use egui::{CentralPanel, Visuals};
use egui_plotter::{Chart, MouseConfig};
use plotters::prelude::*;

extern crate nalgebra as na;

use crate::tag::locator;
use crate::tag::tagged_object::{TagIndex, TagLocation};
use crate::visualize::utils::generate_random_color;

const TAG_CORNERS: [na::Point3<f64>; 5] = [
    na::Point3::new(-1.0, -1.0, 0.0),
    na::Point3::new(1.0, -1.0, 0.0),
    na::Point3::new(1.0, 1.0, 0.0),
    na::Point3::new(-1.0, 1.0, 0.0),
    na::Point3::new(-1.0, -1.0, 0.0),
];

/// A chart to visualize the located objects.
///
/// The `'a` lifetime marker indicates the lifetime of located objects to be visualized.
pub struct VisualizeChart<'a> {
    chart: Chart<Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>>,
    fps: f64,
}

fn to_f64_range(range: &RangeInclusive<i32>) -> Range<f64> {
    range.start().as_f64()..range.end().as_f64()
}

impl<'a> VisualizeChart<'a> {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        object_map: HashMap<String, Vec<(TagIndex, TagLocation)>>,
        located_objects: Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>,
        fps: f64,
    ) -> Self {
        let context = &cc.egui_ctx;
        context.set_visuals(Visuals::light()); // Set to light theme
        let chart = Chart::new(located_objects.clone())
            .mouse(MouseConfig::enabled())
            .pitch(0.7)
            .yaw(0.7)
            .builder_cb(Box::new(move |area, transform, data| {
                let x_axis = -5..=5;
                let y_axis = -5..=5;
                let z_axis = -5..=5;

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

                // plot all located objects
                let located_objects_lock = data.0.lock().unwrap();
                for (name, loc) in located_objects_lock.name_map() {
                    if let Some(object) = object_map.get(*name) {
                        // Get the color of the located object
                        let color = generate_random_color(name);
                        // plot all tags
                        for (_, tag_loc) in object {
                            chart
                                .draw_series(LineSeries::new(
                                    TAG_CORNERS.iter().map(|point| {
                                        let point1 = tag_loc.0.transform_point(point);
                                        let point2 = loc.transform_point(&point1);
                                        (point2.x, point2.y, point2.z)
                                    }),
                                    &color,
                                ))
                                .unwrap();
                        }
                    }
                }
            }));

        Self { chart, fps }
    }
}

impl<'a> eframe::App for VisualizeChart<'a> {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        CentralPanel::default().show(ctx, |ui| {
            self.chart.draw(ui);
        });

        thread::sleep(Duration::from_secs_f64(1.0 / self.fps));
        ctx.request_repaint();
    }
}
