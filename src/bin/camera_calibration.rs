use clap::Parser;
use opencv::core::*;
use opencv::{
    calib3d, highgui, imgproc,
    videoio::{self, VideoCaptureTrait},
};

#[derive(Parser, Debug)]
#[command(
    name = "xDIMScreen camera calibration program",
    version = "0.1.0",
    about = "Calibrate your camera.",
    long_about = r#"This program is for calibrating your camera.

To use the camera calibration program, please:

1. Prepare the camera you want to calibrate and connect it to your computer.
2. Prepare a calibration board with chessboard pattern. Make sure the board is flat and does not bend, or it might impact the calibration result.

After launching the program, you should see a window on your screen. Then, move the calibration board so that the camera captures all corners on the board, and press ENTER on your keyboard to take a picture. Repeat this procedure as many times as needed.

When enough pictures are taken, you can press ESC on your keyboard. This should destroy the window and print a message on the command line, which includes the camera's calibrated parameters."#
)]
struct Args {
    /// The device index of the camera to calibrate. Laptop's builtin camera is usually at index 0.
    #[arg(short, long, default_value_t = 0)]
    cam_id: i32,

    /// The camera resolution's X component.
    #[arg(long, default_value_t = 1920)]
    res_x: u32,

    /// The camera resolution's Y component.
    #[arg(long, default_value_t = 1080)]
    res_y: u32,

    /// The chessboard pattern's width (number of points on X direction).
    #[arg(long)]
    board_x: i32,

    /// The chessboard pattern's height (number of points on Y direction).
    #[arg(long)]
    board_y: i32,

    /// The size of each square on the chessboard. You can choose an arbitrary
    /// unit of length, but the returned camera matrix will be in the same
    /// unit as your unit of choice. Usually we use millimeter as the unit
    /// of length for this field.
    #[arg(short, long)]
    square_size: f32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    highgui::named_window("calibration", highgui::WINDOW_AUTOSIZE)?;
    let mut cam = videoio::VideoCapture::new(args.cam_id, videoio::CAP_ANY)?; // 0 is the default camera
    cam.set(videoio::CAP_PROP_FRAME_WIDTH, args.res_x as f64)?;
    cam.set(videoio::CAP_PROP_FRAME_HEIGHT, args.res_y as f64)?;
    println!("Camera {} started.", args.cam_id);

    let chessboard_pattern_size = Size2i::new(args.board_x, args.board_y);
    let mut object_points = Vector::<Vector<Point3f>>::new();
    let mut image_points = Vector::<Vector<Point2f>>::new();

    let mut mask = Mat::zeros(args.res_y as i32, args.res_x as i32, CV_8UC3)?.to_mat()?;
    let mut taken_picture = false; // whether the user took picture on the last frame

    let (camera_mat, dist_coeff) = loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        let frame_size = frame.size()?;
        if frame_size.width != args.res_x as i32 || frame_size.height != args.res_y as i32 {
            return Err(format!(
                "Frame size mismatch! Expected {}x{}, got {}x{}.",
                args.res_x, args.res_y, frame_size.width, frame_size.height,
            )
            .into());
        }

        let mut gray = Mat::default();
        imgproc::cvt_color(
            &frame,
            &mut gray,
            imgproc::COLOR_BGR2GRAY,
            0,
            opencv::core::AlgorithmHint::ALGO_HINT_ACCURATE,
        )?;

        // find chessboard corner
        let mut corners = Vector::<Point2f>::new();
        let found_chessboard = calib3d::find_chessboard_corners(
            &gray,
            chessboard_pattern_size,
            &mut corners,
            calib3d::CALIB_CB_ADAPTIVE_THRESH
                | calib3d::CALIB_CB_NORMALIZE_IMAGE
                | calib3d::CALIB_CB_FAST_CHECK,
        )?;
        if found_chessboard {
            imgproc::corner_sub_pix(
                &gray,
                &mut corners,
                Size2i::new(11, 11),
                Size2i::new(-1, -1),
                TermCriteria::new(TermCriteria_EPS | TermCriteria_COUNT, 40, 0.001)?,
            )?;
            calib3d::draw_chessboard_corners(
                &mut frame,
                chessboard_pattern_size,
                &corners,
                found_chessboard,
            )?;
        }

        // show image and wait for input
        let masked_frame = (frame + mask.clone() * 0.2).into_result()?;
        if taken_picture {
            highgui::imshow("calibration", &(masked_frame * 2.0).into_result()?)?; // lighten the image for one frame to simulate shutter effect
        } else {
            highgui::imshow("calibration", &masked_frame)?;
        }
        taken_picture = false;
        let key = highgui::wait_key(10)?;
        if key == 27 {
            // Esc pressed. Print the calibration result and end the program.
            if object_points.len() == 0 {
                return Err("No images taken. Terminate.".into());
            }
            let mut camera_mat = Mat::default();
            let mut dist_coeff = Mat::default();
            let mut rvecs = Vector::<Mat>::new();
            let mut tvecs = Vector::<Mat>::new();
            let reprojection_error = calib3d::calibrate_camera(
                &object_points,
                &image_points,
                Size2i::new(args.res_x as i32, args.res_y as i32),
                &mut camera_mat,
                &mut dist_coeff,
                &mut rvecs,
                &mut tvecs,
                0,
                TermCriteria::default()?,
            )?;
            println!("Calibration completed.");
            println!("Camera mat: {:?}", camera_mat);
            println!("Distortion coefficients: {:?}", dist_coeff);
            println!(
                "Reprojection error: {} ({})",
                reprojection_error,
                if reprojection_error < 0.2 {
                    "VERY GOOD"
                } else if reprojection_error < 0.5 {
                    "GOOD"
                } else if reprojection_error < 1.0 {
                    "FINE"
                } else if reprojection_error < 2.0 {
                    "BAD"
                } else {
                    "VERY BAD"
                }
            );
            break (camera_mat, dist_coeff);   // return the camera matrix and distortion coefficients
        } else if key == 10 || key == 13 {
            // Enter pressed. Take a picture and store it in the array.
            if corners.len() != (args.board_x * args.board_y) as usize {
                println!(
                    "This image does not contain the correct number of corners. Corners needed: {}x{}={}.",
                    args.board_x,
                    args.board_y,
                    args.board_x * args.board_y
                );
                continue;
            }

            // draw the chessboard area on the color mask
            let mut points = Vector::<Point>::with_capacity(4);
            points.push(corners.get(0)?.to().unwrap());
            points.push(corners.get(args.board_x as usize - 1)?.to().unwrap());
            points.push(
                corners
                    .get((args.board_x * args.board_y - 1) as usize)?
                    .to()
                    .unwrap(),
            );
            points.push(
                corners
                    .get((args.board_x * (args.board_y - 1)) as usize)?
                    .to()
                    .unwrap(),
            );
            imgproc::fill_poly(
                &mut mask,
                &points,
                Scalar::new(0.0, 255.0, 0.0, 0.0),
                imgproc::LINE_AA,
                0,
                Point::default(),
            )?;

            // add corners to object_points and image_points
            let mut object_corners =
                Vector::<Point3f>::with_capacity((args.board_x * args.board_y) as usize);
            for i in 0..args.board_x {
                for j in 0..args.board_y {
                    object_corners.push(Point3f::new(
                        i as f32 * args.square_size,
                        j as f32 * args.square_size,
                        0.0,
                    ));
                }
            }
            object_points.push(object_corners);
            image_points.push(corners);
            println!(
                "Image captured. Total number of images: {}.",
                object_points.len()
            );

            // update state variables
            taken_picture = true;
        }
    };
    highgui::destroy_all_windows()?;

    // display the undistorted image based on the calibrated parameters
    let image_size = Size::new(args.res_x as i32, args.res_y as i32);
    let new_cam_matrix = calib3d::get_optimal_new_camera_matrix(
        &camera_mat,
        &dist_coeff,
        image_size,
        1.0,
        image_size,
        None,
        false,
    )?;
    let mut map_x = Mat::default();
    let mut map_y = Mat::default();
    calib3d::init_undistort_rectify_map(
        &camera_mat,
        &dist_coeff,
        &Mat::default(),
        &new_cam_matrix,
        image_size,
        CV_32FC1,
        &mut map_x,
        &mut map_y,
    )?;

    highgui::named_window("undistorted image", highgui::WINDOW_AUTOSIZE)?;
    loop {
        let mut frame = Mat::default();
        cam.read(&mut frame)?;
        let mut undistorted = Mat::default();
        imgproc::remap(
            &frame,
            &mut undistorted,
            &map_x,
            &map_y,
            imgproc::INTER_LINEAR,
            BORDER_CONSTANT,
            Scalar::default(),
        )?;
        highgui::imshow("undistorted image", &undistorted)?;
        let key = highgui::wait_key(10)?;
        if key > 0 && key != 255 {
            break;
        }
    }
    Ok(())
}
