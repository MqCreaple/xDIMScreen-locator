# xDIMScreen Locator

## How to Install?

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

### Install `apriltag`

TODO

### Build This Project

Find a path to clone this project:

```bash
git clone _______
cd xDIMScreen-locator
cargo build
```
