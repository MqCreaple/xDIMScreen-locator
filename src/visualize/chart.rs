use std::collections::HashMap;
use std::ops::Range;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;
use std::{f64, thread};

use egui::{CentralPanel, SidePanel, Visuals};
use egui_plotter::{Chart, EguiBackend, MouseConfig};
use plotters::coord::ranged3d::Cartesian3d;
use plotters::coord::types::RangedCoordf64;
use plotters::element::PointCollection;
use plotters::prelude::*;
use statrs::distribution::{self, ContinuousCDF};

extern crate nalgebra as na;

use crate::camera::CameraProperty;
use crate::tag::locator::{self, TAG_CORNERS, TaggedObjectLocator};
use crate::tag::tagged_object::{TagIndex, TagLocation};
use crate::visualize::utils::generate_random_color;

/// A chart to visualize the located objects.
///
/// The `'a` lifetime marker indicates the lifetime of located objects to be visualized.
pub struct VisualizeChart<'a> {
    main_chart: Chart<Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>>,
    axis_angle_chart: Chart<Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>>,
    fps: f64,
}

impl<'a> VisualizeChart<'a> {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        camera: CameraProperty,
        object_map: HashMap<String, Vec<(TagIndex, TagLocation)>>,
        located_objects: Arc<(Mutex<locator::LocatedObjects<'a>>, Condvar)>,
        fps: f64,
    ) -> Self {
        let context = &cc.egui_ctx;
        context.set_visuals(Visuals::light()); // Set to light theme

        let std_normal_distr = distribution::Normal::standard();
        const CONFIDENCE_LEVEL: f64 = 0.95;
        let cbrt_confidence_lvl = CONFIDENCE_LEVEL.cbrt();
        let ellipsoid_scale = std_normal_distr.inverse_cdf(cbrt_confidence_lvl); // these are for plotting the ellipsoid

        let camera_clone = camera.clone();
        let object_map_clone = object_map.clone();
        let main_chart = Chart::new(located_objects.clone())
            .mouse(MouseConfig::enabled())
            .pitch(0.7)
            .yaw(0.7)
            .builder_cb(Box::new(move |area, transform, data| {
                let x_axis = -5.0..5.0;
                let y_axis = -5.0..5.0;
                let z_axis = -5.0..5.0;
                let mut chart = ChartBuilder::on(&area)
                    .caption("Located Objects", ("sans-serif", 16))
                    .build_cartesian_3d(
                        x_axis.clone(),
                        y_axis.clone(),
                        z_axis.clone(),
                    )
                    .unwrap();
                chart.with_projection(|mut pb| {
                    pb.yaw = transform.yaw;
                    pb.pitch = transform.pitch;
                    pb.scale = transform.zoom;
                    pb.into_matrix()
                });
                Self::plot_axes(&mut chart, x_axis, y_axis, z_axis);

                // plot all located objects

                let located_objects_lock = data.0.lock().unwrap();
                for (name, loc) in located_objects_lock.name_map() {
                    if let Some(object) = object_map_clone.get(*name) {
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
                        // plot an ellipsoid representing the confidence regions of the located objects
                        let cov_mat = TaggedObjectLocator::calculate_covariance(
                            camera_clone.camera_mat_na().unwrap(),
                            object.iter().map(|(_, b)| b.clone()),
                            *loc,
                            (2.0, 2.0), // variance of 2 pixel
                        )
                        .unwrap();
                        let cov_mat = cov_mat.try_inverse().unwrap();
                        let a = cov_mat.fixed_view::<3, 3>(0, 0).clone_owned();
                        let b = cov_mat.fixed_view::<3, 3>(0, 3).clone_owned();
                        let bt = cov_mat.fixed_view::<3, 3>(3, 0).clone_owned();
                        let c = cov_mat.fixed_view::<3, 3>(3, 3).clone_owned();
                        Self::plot_ellipsoid(
                            &mut chart,
                            loc.translation.vector,
                            a - b * c.try_inverse().unwrap() * bt,
                            ellipsoid_scale,
                            64,
                            &BLACK,
                        )
                        .unwrap();
                    }
                }
            }));

        let axis_angle_chart = Chart::new(located_objects.clone())
            .mouse(MouseConfig::enabled())
            .pitch(0.7)
            .yaw(0.7)
            .builder_cb(Box::new(move |area, transform, data| {
                let x_axis = -3.5..3.5;
                let y_axis = -3.5..3.5;
                let z_axis = -3.5..3.5;
                let mut chart = ChartBuilder::on(&area)
                    .caption("Axis Angles", ("sans-serif", 16))
                    .build_cartesian_3d(
                        x_axis.clone(),
                        y_axis.clone(),
                        z_axis.clone(),
                    )
                    .unwrap();
                chart.with_projection(|mut pb| {
                    pb.yaw = transform.yaw;
                    pb.pitch = transform.pitch;
                    pb.scale = transform.zoom;
                    pb.into_matrix()
                });
                Self::plot_axes(&mut chart, x_axis, y_axis, z_axis);

                // plot the sphere of radius PI, indicating the range of axis angles
                Self::plot_ellipsoid(
                    &mut chart,
                    na::Vector3::default(),
                    na::Matrix3::identity(),
                    f64::consts::PI,
                    64,
                    &BLACK,
                ).unwrap();

                // plot the axis angle of all objects
                let located_objects_lock = data.0.lock().unwrap();
                for (name, loc) in located_objects_lock.name_map() {
                    if let Some(object) = object_map.get(*name) {
                        let color = generate_random_color(name);
                        let axis_angle = loc.rotation.scaled_axis();
                        chart.draw_series(LineSeries::new([(0.0, 0.0, 0.0), (axis_angle.x, axis_angle.y, axis_angle.z)], &color)).unwrap();
                        // draw an ellipsoid to represent the covariance
                        let cov_mat = TaggedObjectLocator::calculate_covariance(
                            camera.camera_mat_na().unwrap(),
                            object.iter().map(|(_, b)| b.clone()),
                            *loc,
                            (2.0, 2.0), // variance of 2 pixel
                        )
                        .unwrap();
                        let cov_mat = cov_mat.try_inverse().unwrap();
                        let a = cov_mat.fixed_view::<3, 3>(0, 0).clone_owned();
                        let b = cov_mat.fixed_view::<3, 3>(0, 3).clone_owned();
                        let bt = cov_mat.fixed_view::<3, 3>(3, 0).clone_owned();
                        let c = cov_mat.fixed_view::<3, 3>(3, 3).clone_owned();
                        Self::plot_ellipsoid(
                            &mut chart,
                            axis_angle,
                            c - bt * a.try_inverse().unwrap() * b,
                            ellipsoid_scale,
                            64,
                            &color,
                        )
                        .unwrap();
                    }
                }
            }));
        Self { main_chart, axis_angle_chart, fps }
    }

    fn plot_axes(
        chart: &mut ChartContext<'_, EguiBackend<'_>, Cartesian3d<RangedCoordf64, RangedCoordf64, RangedCoordf64>>,
        x_axis: Range<f64>,
        y_axis: Range<f64>,
        z_axis: Range<f64>,
    ) {

        chart
            .draw_series(LineSeries::new(
                [(x_axis.start, 0., 0.), (x_axis.end, 0., 0.)],
                &RED,
            ))
            .unwrap()
            .label("x axis");
        chart
            .draw_series(LineSeries::new(
                [(0., y_axis.start, 0.), (0., y_axis.end, 0.)],
                &GREEN,
            ))
            .unwrap()
            .label("y axis");
        chart
            .draw_series(LineSeries::new(
                [(0., 0., z_axis.start), (0., 0., z_axis.end)],
                &BLUE,
            ))
            .unwrap()
            .label("z axis");
    }

    /// Draw an ellipsoid centered at `center` with `mat` describing its shape.
    ///
    /// The ellipsoid's formula can be written as:
    ///
    /// $$(x - c)^T \mathbf{Q} (x - c) = S^2$$
    ///
    /// Where $c$ is `center`, $Q$ is `mat`, and $S$ is `scale`.
    ///
    /// The matrix $Q$ does not need to be symmetric, as this function will replace $Q$ with
    /// $\frac{1}{2}(Q + Q^T)$.
    ///
    /// `resolution` defines the number of points to sample on each arc.
    fn plot_ellipsoid<DB: DrawingBackend, CT: CoordTranslate, S: Into<ShapeStyle> + Clone>(
        chart: &mut ChartContext<'_, DB, CT>,
        center: na::Vector3<f64>,
        mut mat: na::Matrix3<f64>,
        scale: f64,
        resolution: usize,
        style: S,
    ) -> Result<(), DrawingAreaErrorKind<DB::ErrorType>>
    where
        for<'b> &'b DynElement<'static, DB, (f64, f64, f64)>:
            PointCollection<'b, <CT as plotters::coord::CoordTranslate>::From>,
    {
        let dt = 2.0 * f64::consts::PI / (resolution as f64);
        mat = (mat + mat.transpose()) * 0.5;
        let eigen = mat.symmetric_eigen();
        for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
            let vec1 = eigen.eigenvectors.column(i);
            let val1 = unsafe { *eigen.eigenvalues.get_unchecked(i) };
            let sqrtval1 = val1.sqrt();
            let vec2 = eigen.eigenvectors.column(j);
            let val2 = unsafe { *eigen.eigenvalues.get_unchecked(j) };
            let sqrtval2 = val2.sqrt();
            chart.draw_series(LineSeries::new(
                (0..(resolution + 1)).map(|t| {
                    let t1 = (t as f64) * dt;
                    let vec = center
                        + vec1 * scale / sqrtval1 * t1.cos()
                        + vec2 * scale / sqrtval2 * t1.sin();
                    unsafe {
                        (
                            *vec.get_unchecked(0),
                            *vec.get_unchecked(1),
                            *vec.get_unchecked(2),
                        )
                    }
                }),
                style.clone(),
            ))?;
        }
        Ok(())
    }
}

impl<'a> eframe::App for VisualizeChart<'a> {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        SidePanel::right("axis_angle_panel")
        .resizable(true)
        .show(ctx, |ui| {
            ui.separator();
            self.axis_angle_chart.draw(ui);
        });
        CentralPanel::default()
        .show(ctx, |ui| {
            self.main_chart.draw(ui);
        });

        thread::sleep(Duration::from_secs_f64(1.0 / self.fps));
        ctx.request_repaint();
    }
}
