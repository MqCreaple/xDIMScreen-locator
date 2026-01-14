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

### Calibrate Your Camera (If You Haven't)

You need to prepare a chessboard pattern for camera calibration. You can print out the chessboard pattern (which should be easily available online) on a paper, count the number of corners on x and y directions (NOT the number of grids), and stick it to a flat surface that can be held in front of your camera.

Run the following command, replacing `<BOARD-X>` and `<BOARD-Y>` with the number of corners on x and y directions, and `<SQUARE-SIZE>` with the width of each square (in mm):

```bash
cargo run --bin camera_calibration --release -- --board-x <BOARD-X> --board-y <BOARD-Y> --square-size <SQUARE-SIZE>
```

This should pop up a new window showing the video stream from your camera. If you are building it for the first time, it might take 10 to 30 minutes.

Holding the chessboard pattern in front of your camera, and you should see the video on the window showing a collection of dots on each corner, connected by lines. You can press `Enter` to take a picture, after which a green region covering the dots should appear on the screen.

You should take as many photos as possible (at least 20), and ensure that the green region covers most of the image and are captured with a variety of angles. After the photos are taken, press `Esc` to exit. The console should then print out the camera matrix, distortion coefficient, and reprojection error.

### Run xDIMScreen Locator

After finishing all steps above, you can directly run the locator by:

```bash
cargo run --bin xDIMScreen_locator --release --cam-fov-x <CAM-FOV-X>
```

Where `<CAM-FOV-X>` is the camera's field of view on x direction, measured in degrees. The camera's FOV can be calculated from the camera matrix. Alternatively, you can provide the camera's FOV on y direction by passing it to the parameter: `--cam-fov-y <CAM-FOV-Y>`.
