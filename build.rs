use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let project_root = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={}", Path::new(&project_root).join("ext").join("apriltag").join("build").join("Release").display());
    println!("cargo:rustc-link-lib=static=apriltag");

    let apriltag_bindings = bindgen::Builder::default()
        .header("apriltag-wrapper.h")
        .allowlist_file(r"^(.*apriltag\.h|.*apriltag_pose\.h|.*tag.*\.h|.*image_.*|.*homography\.h|.*zarray\.h)$")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings for apriltag!");
    
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    apriltag_bindings
        .write_to_file(out_path.join("apriltag-bindings.rs"))
        .expect("Couldn't write bindings!");
}