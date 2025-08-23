# xDIMScreen Locator

## How to Install?

### Clone this project

```bash
git clone https://github.com/MqCreaple/xDIMScreen-locator
cd xDIMScreen-locator
git submodule update --init --recursive
```

### Install `opencv`

Since this project uses the [`opencv`](https://docs.rs/opencv/latest/opencv/) package, you need to first install OpenCV on your computer. Please follow the documentation on opencv-rust: <https://github.com/twistedfall/opencv-rust/blob/master/INSTALL.md>.

If you are using Windows, you need to set `OPENCV_LINK_LIBS`, `OPENCV_LINK_PATHS`, and `OPENCV_INCLUDE_PATHS` in your environmental variables. Please check according to the documentation of [opencv-rust](https://github.com/twistedfall/opencv-rust). You can do this by creating a file at `.cargo/config.toml`:

```toml
[env]
OPENCV_LINK_LIBS = "opencv_world4120"
OPENCV_LINK_PATHS = "D:\\opencv\\build\\x64\\vc16\\lib"
OPENCV_INCLUDE_PATHS = "D:\\opencv\\build\\include"
```

After adding everything to the environmental variables, make sure to build the crate again.

```bash
cargo build
```

If you encounter any problem related to OpenCV, please refer to the [troubleshooting page](https://github.com/twistedfall/opencv-rust/blob/master/TROUBLESHOOTING.md) at opencv-rust.

### Build `apriltag`

```bash
cd ext/apriltag
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_WRAPPER=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF ..
cmake --build . --config Release
```

After building the library, you need to add `ext/apriltag/build/` or `ext/apriltag/build/Release/` (depending on the compiler you used) to your operating system's `PATH` variable and then restart your computer, or the program might not find the correct dll to link.

### Run xDIMScreen Locator

After finishing all steps above, you can directly run the locator by:

```bash
cargo run --bin xDIMScreen_locator --release
```

This will automatically build the project in release mode and start running after the build finishes. If you are building it for the first time, it might take 10 to 30 minutes.
