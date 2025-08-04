use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
};

use xDIMScreen_locator::{
    camera::CameraProperty,
    tag::{
        apriltag::{ApriltagDetection, ApriltagFamily, ApriltagFamilyType, apriltag_binding},
        locator::TaggedObjectLocator,
        tagged_object::TaggedObject,
    },
};

#[test]
fn test_locator() {
    let camera_prop =
        CameraProperty::new((1920, 1080), (None, Some(f64::to_radians(50.0))), None).unwrap();

    let mut locator = TaggedObjectLocator::new(camera_prop);
    let simple_obj = TaggedObject::new_simple("simple", ApriltagFamily::Tag36h11, 0, 1.0);
    locator.add(&simple_obj).unwrap();
    let locator_results = Arc::new(Mutex::new(BTreeMap::new()));

    let family_tag36h11 = ApriltagFamilyType::new(ApriltagFamily::Tag36h11);
    let dummy_h_matd = unsafe { apriltag_binding::matd_create(2, 2) };
    let detection_raw = unsafe {
        libc::malloc(std::mem::size_of::<apriltag_binding::apriltag_detection>())
            as *mut apriltag_binding::apriltag_detection
    };
    unsafe {
        (*detection_raw).family = family_tag36h11.c_type;
        (*detection_raw).id = 0;
        (*detection_raw).hamming = 2;
        (*detection_raw).decision_margin = 0.1;
        (*detection_raw).H = dummy_h_matd;

        // the tag is located at the center of the camera. Its side length is 20 pixels seen by the camera.
        (*detection_raw).c = [960.0, 540.0];
        (*detection_raw).p = [
            [950.0, 550.0],
            [970.0, 550.0],
            [970.0, 530.0],
            [950.0, 530.0],
        ];
    }
    let detection = unsafe { ApriltagDetection::new_from_raw(detection_raw) };

    locator
        .locate_objects(&[detection], locator_results.clone())
        .unwrap();

    let result_lock = locator_results.lock().unwrap();

    // object detector should be able to detect the object 'simple'
    assert!(result_lock.contains_key("simple"));

    let simple_tag_location = result_lock.get("simple").unwrap();
    // 'simple' should not have any rotation
    assert!(f64::abs(simple_tag_location.rotation.i) <= 1e-5);
    assert!(f64::abs(simple_tag_location.rotation.j) <= 1e-5);
    assert!(f64::abs(simple_tag_location.rotation.k) <= 1e-5);
    assert!(f64::abs(simple_tag_location.rotation.w - 1.0) <= 1e-5);

    // 'simple' should not have any translation on x or y direction
    assert!(f64::abs(simple_tag_location.translation.x) <= 1e-5);
    assert!(f64::abs(simple_tag_location.translation.y) <= 1e-5);
    // the distance (z translation) of 'simple' from the camera should be approximately 57.9 unit
    assert!(f64::abs(simple_tag_location.translation.z - 57.90) <= 0.01);
}
