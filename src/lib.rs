use opencv::prelude::*;
use opencv::core::{Rect, Size, TermCriteria_MAX_ITER, Vector, Point2f, Point3f};
use opencv::calib3d;
use opencv::imgproc;
use opencv::imgcodecs;
use std::error::Error;


struct PointCloud2Msg{
    header: String,
    height: i32,
    width: i32,
    fields: Vec<i32>,
    is_bigendian: bool,
    point_step: i32,
    row_step: i32,
    data: Vec<i8>, // row_step * height
    is_dense: bool

}





fn get_size(image: Mat) -> (i32, i32) 
{
    let rows = image.rows() as i32;
    let cols = image.cols() as i32;

    (rows, cols)
}


fn establish_rois_zone(scope: Mat) -> (Rect, Rect)
{

    let (rows, cols) =  get_size(scope);
    let left_roi = Rect::new(0,0,cols / 2, rows);
    let right_roi = Rect::new(cols / 2, 0, cols - cols / 2, rows);
    (left_roi, right_roi)
}

fn call_roi(image: &Mat, left_roi:  &Rect, right_roi:  &Rect) -> Result<(Mat, Mat), Error> {
    let mut left = Mat::roi(image, *left_roi)?;
    let mut right = Mat::roi(image, *right_roi)?;
    let left = left.clone_pointee();
    let right = right.clone_pointee();

    Ok((left, right))
}




fn stereo_calibrate(camera1: Mat, camera2: Mat, images1: Vec<Mat>, images2: Vec<Mat>, points: ) -> Result<(Mat, Mat, Mat, Mat), Error> {
     // Chessboard dimensions (width x height of internal corners)
     let chessboard_size = Size::new(9, 6);

     // Square size of the chessboard in your desired unit (e.g., meters or millimeters)
     let square_size = 0.025; // 25 mm or 0.025 meters
 
     // Prepare object points (3D points in real-world space for one chessboard)
     let mut obj_points = Vec::<Point3f>::new();
     for i in 0..chessboard_size.height {
         for j in 0..chessboard_size.width {
             obj_points.push(Point3f::new(j as f32 * square_size, i as f32 * square_size, 0.0));
         }
     }
 
     let mut object_points = Vec::<Mat>::new();
     let mut image_points_left = Vec::<Mat>::new();
     let mut image_points_right = Vec::<Mat>::new();
 
     // Load chessboard images for the left and right cameras
     let left_images = vec!["left01.jpg", "left02.jpg", "left03.jpg"];
     let right_images = vec!["right01.jpg", "right02.jpg", "right03.jpg"];
 
     for (left_img_path, right_img_path) in left_images.iter().zip(right_images.iter()) {
         let left_img = imgcodecs::imread(left_img_path, imgcodecs::IMREAD_GRAYSCALE)?;
         let right_img = imgcodecs::imread(right_img_path, imgcodecs::IMREAD_GRAYSCALE)?;
 
         let mut corners_left = Vec::<Point2f>::new();
         let mut corners_right = Vec::<Point2f>::new();
 
         // Detect chessboard corners
         let found_left = calib3d::find_chessboard_corners(
             &left_img,
             chessboard_size,
             &mut corners_left,
             calib3d::CALIB_CB_ADAPTIVE_THRESH | calib3d::CALIB_CB_NORMALIZE_IMAGE,
         )?;
         let found_right = calib3d::find_chessboard_corners(
             &right_img,
             chessboard_size,
             &mut corners_right,
             calib3d::CALIB_CB_ADAPTIVE_THRESH | calib3d::CALIB_CB_NORMALIZE_IMAGE,
         )?;
 
         if found_left && found_right {
             // Refine corner positions
             imgproc::corner_sub_pix(
                 &left_img,
                 &mut corners_left,
                 Size::new(11, 11),
                 Size::new(-1, -1),
                 TermCriteria::new(
                     opencv::core::TermCriteriaType::COUNT + opencv::core::TermCriteriaType::EPS,
                     30,
                     0.001,
                 )?,
             )?;
             imgproc::corner_sub_pix(
                 &right_img,
                 &mut corners_right,
                 Size::new(11, 11),
                 Size::new(-1, -1),
                 TermCriteria::new(
                     opencv::core::TermCriteriaType::COUNT + opencv::core::TermCriteriaType::EPS,
                     30,
                     0.001,
                 )?,
             )?;
 
             image_points_left.push(Mat::from_slice(&corners_left.to_vec())?);
             image_points_right.push(Mat::from_slice(&corners_right.to_vec())?);
             object_points.push(Mat::from_slice(&obj_points.to_vec())?);
         }
     }
 
     // Calibration outputs
     let mut camera_matrix_left = Mat::eye(3, 3, opencv::core::CV_64F)?;
     let mut dist_coeffs_left = Mat::zeros(5, 1, opencv::core::CV_64F)?;
     let mut camera_matrix_right = Mat::eye(3, 3, opencv::core::CV_64F)?;
     let mut dist_coeffs_right = Mat::zeros(5, 1, opencv::core::CV_64F)?;
 
     let mut rotation = Mat::default();
     let mut translation = Mat::default();
     let mut essential_matrix = Mat::default();
     let mut fundamental_matrix = Mat::default();
 
     // Stereo calibration
     calib3d::stereo_calibrate(
         &object_points,
         &image_points_left,
         &image_points_right,
         &mut camera_matrix_left,
         &mut dist_coeffs_left,
         &mut camera_matrix_right,
         &mut dist_coeffs_right,
         chessboard_size,
         &mut rotation,
         &mut translation,
         &mut essential_matrix,
         &mut fundamental_matrix,
         calib3d::CALIB_FIX_INTRINSIC,
         TermCriteria::new(
             opencv::core::TermCriteriaType::COUNT + opencv::core::TermCriteriaType::EPS,
             100,
             1e-5,
         )?,
     )?;
 
     // Output the calibration results
     println!("Left Camera Matrix: {:?}", camera_matrix_left);
     println!("Right Camera Matrix: {:?}", camera_matrix_right);
     println!("Rotation: {:?}", rotation);
     println!("Translation: {:?}", translation);
 
     Ok(())
 }

// Stereo rectification
fn stereo_rectify(
    r: Mat, 
    t: Mat, 
    camera1: Mat, 
    camera2: Mat, 
    roi1: &mut Rect, 
    roi2: &mut Rect) -> Result<(Mat, Mat, Mat, Mat, Mat), Error> {
    
    let mut r1 = Mat::default();
    let mut r2 = Mat::default();
    let mut p1 = Mat::default();
    let mut p2 = Mat::default();
    let mut q = Mat::default();
    let alpha: f64 = 0.0;
    let flag = 0;
    let new_img_size = &camera1.size();
    
    let camera_matrix_left = Mat::from_slice_2d(&[
        [700.0, 0.0, 640.0],
        [0.0, 700.0, 360.0],
        [0.0, 0.0, 1.0],
    ])?;
    let dist_coeffs_left = Mat::from_slice(&[0.1, -0.25, 0.0, 0.0])?;
    let camera_matrix_right = Mat::from_slice_2d(&[
        [700.0, 0.0, 640.0],
        [0.0, 700.0, 360.0],
        [0.0, 0.0, 1.0],
    ])?;
    let dist_coeffs_right = Mat::from_slice(&[0.1, -0.25, 0.0, 0.0])?;

    // Rotation and translation between cameras
    let rotation = Mat::from_slice_2d(&[
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])?;
    let translation = Mat::from_slice(&[0.1, 0.0, 0.0])?;

    
    
    
    let value   = opencv::calib3d::stereo_rectify(
        &camera_matrix_left,
        &dist_coeffs_left,
        &camera_matrix_right,
        &dist_coeffs_right,
        camera1.size()?,
        &rotation,
        &translation,
        &mut r1,
        &mut r2,
        &mut p1,
        &mut p2,
        &mut q,
        flag,
        alpha,
        new_img_size.unwrap(),
        roi1,
        roi2
    )?;

    Ok((r1, r2, p1, p2, q))
}

// Generate point cloud from disparity map
fn generate_point_cloud(q: Mat, disparity: Mat) -> Result<Mat, Error> {
    let mut points   = Mat::default();
    opencv::calib3d::reproject_image_to_3d(&disparity, &q, &mut points)?;
    
    Ok(points)
}

// Main function
fn main() -> Result<(), Error> {
    // Capture calibration images for both cameras
    let images1   = capture_calibration_images(&mut cap1)?;
    let images2   = capture_calibration_images(&mut cap2)?;

    // Calibrate each camera
    let (camera_matrix1, dist_coeffs1)   = calibrate_camera(images1, (9, 6), 1.0)?;
    let (camera_matrix2, dist_coeffs2)   = calibrate_camera(images2, (9, 6), 1.0)?;

    // Stereo calibration
    let (r, t, camera_matrix_left, dist_coeffs_left)   = stereo_calibrate(
        (camera_matrix1.clone(), dist_coeffs1.clone()),
        (camera_matrix2.clone(), dist_coeffs2.clone()),
        images1,
        images2,
    )?;

    // Stereo rectification
    let (r1, r2, p1, p2, q)   = stereo_rectify(r, t, camera_matrix_left, camera_matrix_right)?;

    // Now you can use the rectified parameters to process live video

    Ok(())
}