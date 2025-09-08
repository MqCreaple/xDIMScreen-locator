extern crate nalgebra as na;

use std::ops::DerefMut;

use rand::Rng;

use crate::tag::apriltag::{
    ApriltagDetection, ApriltagFamily, ApriltagFamilyType, apriltag_binding,
};

use super::*;

fn project_tag(ret: &mut na::DVector<f64>, object_location: &na::Isometry3<f64>, object: &TaggedObject, camera_mat: &na::Matrix3<f64>) {
    for (j, (_, tag_location)) in object.tags.iter().enumerate() {
        for k in 0..4 {
            let point = camera_mat
                * object_location
                    .transform_point(&tag_location.0.transform_point(&TAG_CORNERS[k]));
            ret[j * 8 + k * 2] = point.x / point.z;
            ret[j * 8 + k * 2 + 1] = point.y / point.z;
        }
    }
}

#[test]
fn test_projection_jacobian() {
    let camera =
        CameraProperty::new((1920, 1080), (None, Some(f64::to_radians(50.0))), None).unwrap();
    let camera_mat = camera.camera_mat_na().unwrap();

    let mut object = TaggedObject::new("test object");
    let tag36h11_family = ApriltagFamilyType::new(apriltag::ApriltagFamily::Tag36h11);
    object.tags.insert(
        TagIndex {
            family: ApriltagFamily::Tag36h11,
            id: 0,
        },
        TagLocation::new(1.0, na::Vector3::default(), na::vector![0.0, 0.0, 0.0]),
    );
    object.tags.insert(
        TagIndex {
            family: ApriltagFamily::Tag36h11,
            id: 1,
        },
        TagLocation::new(1.0, na::Vector3::default(), na::vector![0.0, 1.0, 2.0]),
    );
    object.tags.insert(
        TagIndex {
            family: ApriltagFamily::Tag36h11,
            id: 2,
        },
        TagLocation::new(1.0, na::Vector3::default(), na::vector![-2.0, 1.0, 0.0]),
    );
    let mut locator = TaggedObjectLocator::new(camera);
    locator.add(&object).unwrap();

    let mut rng = rand::rng();
    for _ in 0..100 {
        // simulate a projection from a random location
        let object_location = na::Isometry3::new(
            na::vector![
                rng.random_range(-3.0..3.0),
                rng.random_range(-3.0..3.0),
                rng.random_range(4.0..12.0)
            ],
            na::vector![0.0, 0.05, 0.02],
        );
        let mut detections = Vec::with_capacity(object.tags.len());
        for (index, tag_location) in &object.tags {
            let center = camera_mat
                * object_location.transform_point(
                    &tag_location
                        .0
                        .transform_point(&na::Point3::new(0.0, 0.0, 0.0)),
                );
            let corners = std::array::from_fn(|i| {
                let point = camera_mat
                    * object_location
                        .transform_point(&tag_location.0.transform_point(&TAG_CORNERS[i]));
                [point.x / point.z, point.y / point.z]
            });
            let dummy_h_matd = unsafe { apriltag_binding::matd_create(2, 2) };
            let mut detection_raw = Box::new(apriltag_binding::apriltag_detection {
                family: tag36h11_family.c_type,
                id: index.id,
                hamming: 0,
                decision_margin: 0.0,
                H: dummy_h_matd,
                c: [center.x / center.z, center.y / center.z],
                p: corners,
            });
            let detection = unsafe { ApriltagDetection::new_from_raw(detection_raw.deref_mut()) };
            std::mem::forget(detection_raw);
            detections.push((detection, tag_location.clone()));
        }

        // calculate projection jacobian
        let calculated_projection_jacobian = TaggedObjectLocator::calculate_projection_jacobian(
            locator.camera.camera_mat_na().unwrap(),
            detections.iter().map(|(_, b)| b.clone()),
            object_location,
        )
        .unwrap();

        assert_eq!(
            calculated_projection_jacobian.nrows(),
            8 * object.tags.len()
        );

        // measure the projection jacobian with numerical differentiation
        let mut measured_projection_jacobian =
            na::MatrixXx6::zeros(calculated_projection_jacobian.nrows());
        const EPS: f64 = 1e-4;
        for (i, shift) in [
            na::vector![EPS, 0.0, 0.0],
            na::vector![0.0, EPS, 0.0],
            na::vector![0.0, 0.0, EPS],
        ]
        .into_iter()
        .enumerate()
        {
            // measure translations
            let mut object_location_1 = object_location.clone();
            object_location_1.append_translation_mut(&na::Translation3::from(-shift));
            let mut vec1 = na::DVector::zeros(calculated_projection_jacobian.nrows());
            project_tag(&mut vec1, &object_location_1, &object, &camera_mat);
            let mut object_location_2 = object_location.clone();
            object_location_2.append_translation_mut(&na::Translation3::from(shift));
            let mut vec2 = na::DVector::zeros(calculated_projection_jacobian.nrows());
            project_tag(&mut vec2, &object_location_2, &object, &camera_mat);
            let diff = (vec2 - vec1) / (2.0 * EPS);
            measured_projection_jacobian.set_column(i, &diff);
        }
        for (i, rotate) in [
            na::vector![EPS, 0.0, 0.0],
            na::vector![0.0, EPS, 0.0],
            na::vector![0.0, 0.0, EPS],
        ]
        .into_iter()
        .enumerate()
        {
            // measure rotations
            let mut object_location_1 = object_location.clone();
            object_location_1.rotation = na::UnitQuaternion::from_scaled_axis(object_location_1.rotation.scaled_axis() - rotate);
            let mut vec1 = na::DVector::zeros(calculated_projection_jacobian.nrows());
            project_tag(&mut vec1, &object_location_1, &object, &camera_mat);
            let mut object_location_2 = object_location.clone();
            object_location_2.rotation = na::UnitQuaternion::from_scaled_axis(object_location_2.rotation.scaled_axis() + rotate);
            let mut vec2 = na::DVector::zeros(calculated_projection_jacobian.nrows());
            project_tag(&mut vec2, &object_location_2, &object, &camera_mat);
            let diff = (vec2 - vec1) / (2.0 * EPS);
            measured_projection_jacobian.set_column(i + 3, &diff);
        }

        for i in 0..calculated_projection_jacobian.nrows() {
            for j in 0..3 {
                let measured_i_j = measured_projection_jacobian.get((i, j)).unwrap();
                let calculated_i_j = calculated_projection_jacobian.get((i, j)).unwrap();
                assert!(
                    (measured_i_j - calculated_i_j).abs() <= 0.01 * calculated_i_j.abs(), // less than 1% of relative error
                    "Assertion failed on index {} {}, with the measured value being {} and calculated value being {}",
                    i,
                    j,
                    measured_i_j,
                    calculated_i_j
                );
            }
        }
    }
}
