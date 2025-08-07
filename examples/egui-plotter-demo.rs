use std::{
    error,
    ops::{Range, RangeInclusive},
    thread,
    time::Duration,
};

use egui::{CentralPanel, Visuals};
use egui_plotter::{Chart, MouseConfig};
use plotters::{self, prelude::*};

fn main() -> Result<(), Box<dyn error::Error>> {
    let native_options = eframe::NativeOptions::default(); // rendering on your desktop
    eframe::run_native(
        "Demo Project",
        native_options,
        Box::new(|cc| Ok(Box::new(DemoChart::new(cc)))),
    )?;
    Ok(())
}

struct DemoChart {
    chart: Chart<()>,
}

fn to_f64_range(range: &RangeInclusive<i32>) -> Range<f64> {
    range.start().as_f64()..range.end().as_f64()
}

impl DemoChart {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let context = &cc.egui_ctx;
        context.set_visuals(Visuals::light()); // Set to light theme
        let chart = Chart::new(())
            .mouse(MouseConfig::enabled())
            .pitch(0.7)
            .yaw(0.7)
            .builder_cb(Box::new(|area, transform, _d| {
                let x_axis = -5..=5;
                let y_axis = -5..=5;
                let z_axis = -5..=5;

                let mut chart = ChartBuilder::on(&area)
                    .caption("3D plot", ("sans-serif", 16))
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
                    .draw_series(LineSeries::new(
                        x_axis.map(|x| (x.as_f64(), 0., 0.)),
                        &BLACK,
                    ))
                    .unwrap()
                    .label("x axis");
                chart
                    .draw_series(LineSeries::new(
                        y_axis.map(|y| (0., y.as_f64(), 0.)),
                        &BLACK,
                    ))
                    .unwrap()
                    .label("y axis");
                chart
                    .draw_series(LineSeries::new(
                        z_axis.map(|z| (0., 0., z.as_f64())),
                        &BLACK,
                    ))
                    .unwrap()
                    .label("z axis");

                // Draw series here
                chart
                    .draw_series(LineSeries::new(
                        [(0., 0., 1.), (1., 2., 3.), (3., 4., 2.)].into_iter(),
                        &BLUE,
                    ))
                    .unwrap();
            }));

        Self { chart }
    }
}

impl eframe::App for DemoChart {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        const FPS: f64 = 60.0;

        CentralPanel::default().show(ctx, |ui| {
            self.chart.draw(ui);
        });

        thread::sleep(Duration::from_secs_f64(1.0 / FPS));
        ctx.request_repaint();
    }
}
