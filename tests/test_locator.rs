use std::{
    sync::{Arc, Condvar, Mutex},
    time::SystemTime,
};

use xDIMScreen_locator::{
    camera::CameraProperty,
    tag::{
        apriltag::{ApriltagDetection, ApriltagFamily, ApriltagFamilyType, apriltag_binding},
        locator::{LocatedObjects, TaggedObjectLocator},
        tagged_object::TaggedObject,
    },
};

#[test]
fn test_locator_simple_tag() {
    let camera_prop =
        CameraProperty::new((1920, 1080), (None, Some(f64::to_radians(50.0))), None).unwrap();

    let mut locator = TaggedObjectLocator::new(camera_prop);
    let simple_obj = TaggedObject::new_simple("simple", ApriltagFamily::Tag36h11, 0, 1.0);
    locator.add(&simple_obj).unwrap();
    let locator_results = Arc::new((Mutex::new(LocatedObjects::new()), Condvar::new()));

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
        (*detection_raw).c = [960.0 - 0.5, 540.0 - 0.5];
        (*detection_raw).p = [
            [950.0 - 0.5, 550.0 - 0.5],
            [970.0 - 0.5, 550.0 - 0.5],
            [970.0 - 0.5, 530.0 - 0.5],
            [950.0 - 0.5, 530.0 - 0.5], // move -0.5 pixels to make the center exactly (960, 540)
        ];
    }
    let detection = unsafe { ApriltagDetection::new_from_raw(detection_raw) };

    locator
        .locate_objects(SystemTime::now(), &[detection], locator_results.clone())
        .unwrap();

    let result_lock = locator_results.0.lock().unwrap();

    // object detector should be able to detect the object 'simple'
    assert!(result_lock.name_map().contains_key("simple"));

    let simple_tag_location = result_lock.name_map().get("simple").unwrap();
    const EPS: f64 = 1e-5;
    // 'simple' should not have any rotation
    println!("Rotation: {:?}", simple_tag_location.rotation);
    assert!(
        f64::abs(simple_tag_location.rotation.i) <= EPS,
        "Rotation i is not zero. Value: {}",
        simple_tag_location.rotation.i
    );
    assert!(
        f64::abs(simple_tag_location.rotation.j) <= EPS,
        "Rotation j is not zero. Value: {}",
        simple_tag_location.rotation.j
    );
    assert!(
        f64::abs(simple_tag_location.rotation.k) <= EPS,
        "Rotation k is not zero. Value: {}",
        simple_tag_location.rotation.k
    );
    assert!(
        f64::abs(simple_tag_location.rotation.w - 1.0) <= EPS,
        "Rotation w is not one. Value: {}",
        simple_tag_location.rotation.w
    );

    // 'simple' should not have any translation on x or y direction
    println!("Translation: {:?}", simple_tag_location.translation);
    assert!(
        f64::abs(simple_tag_location.translation.x) <= EPS,
        "Translation x is not zero. Value: {}",
        simple_tag_location.translation.x
    );
    assert!(
        f64::abs(simple_tag_location.translation.y) <= EPS,
        "Translation y is not zero. Value: {}",
        simple_tag_location.translation.y
    );
    // the distance (z translation) of 'simple' from the camera should be approximately 57.85 unit
    assert!(
        f64::abs(simple_tag_location.translation.z - 57.85) <= 0.01,
        "Translation z is not approximately 57.85. Value: {}",
        simple_tag_location.translation.z
    );
}
