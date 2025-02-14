use opencv::prelude::*;
use opencv::core::{Point2f, 
    Point3f, 
    Rect, 
    Size, 
    TermCriteria, 
    TermCriteria_COUNT, 
    TermCriteria_EPS, 
    Vector, 
    NORM_MINMAX,
    CV_16F};
use opencv::calib3d::{self, StereoSGBM_MODE_SGBM};
use opencv::imgproc;
use opencv::imgcodecs;
use opencv::Error;


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


fn establish_rois_zone(scope: &Mat) -> (Rect, Rect)
{
    let scope = scope.clone();
    let (rows, cols) =  get_size(scope);
    let left_roi = Rect::new(0,0,cols / 2, rows);
    let right_roi = Rect::new(cols / 2, 0, cols - cols / 2, rows);
    (left_roi, right_roi)
}

fn call_roi(image: &Mat, left_roi:  &Rect, right_roi:  &Rect) -> Result<(Mat, Mat), Error> {
    let left = Mat::roi(image, *left_roi)?;
    let right = Mat::roi(image, *right_roi)?;
    let left = left.clone_pointee();
    let right = right.clone_pointee();

    Ok((left, right))
}




fn stereo_calibrate() -> Result<(Mat, Mat, Mat, Mat), Error> {
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
 
     let mut object_points = Vector::<Mat>::new();
     let mut image_points_left = Vector::<Mat>::new();
     let mut image_points_right = Vector::<Mat>::new();
 
     // Load chessboard images for the left and right cameras
     let left_images = vec!["left01.jpg", "left02.jpg", "left03.jpg"];
     let right_images = vec!["right01.jpg", "right02.jpg", "right03.jpg"];
 
     for (left_img_path, right_img_path) in left_images.iter().zip(right_images.iter()) {
         let left_img = imgcodecs::imread(left_img_path, imgcodecs::IMREAD_GRAYSCALE)?;
         let right_img = imgcodecs::imread(right_img_path, imgcodecs::IMREAD_GRAYSCALE)?;
 
         let mut corners_left = Vector::<Point2f>::new();
         let mut corners_right = Vector::<Point2f>::new();
 
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
                    TermCriteria_COUNT + TermCriteria_EPS,
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
                    TermCriteria_COUNT + TermCriteria_EPS,
                     30,
                     0.001,
                 )?,
             )?;
 
             image_points_left.push(Mat::from_slice(&corners_left.to_vec())?.clone_pointee());
             image_points_right.push(Mat::from_slice(&corners_right.to_vec())?.clone_pointee());
             object_points.push(Mat::from_slice(&obj_points.to_vec())?.clone_pointee());
         }
     }
 
     // Calibration outputs
     let mut camera_matrix_left =  Mat::default();
     let mut dist_coeffs_left = Mat::default();
     let mut camera_matrix_right = Mat::default();
     let mut dist_coeffs_right = Mat::default();
 
     let mut rotation = Mat::default();
     let mut translation = Mat::default();
     let mut essential_matrix = Mat::default();
     let mut fundamental_matrix = Mat::default();
 
     // Stereo calibration
     opencv::calib3d::stereo_calibrate(
         &mut object_points,
         &mut image_points_left,
         &mut image_points_right,
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
             TermCriteria_COUNT + TermCriteria_EPS,
             100,
             1e-5,
         )?,
     )?;
 
     // Output the calibration results
     println!("Left Camera Matrix: {:?}", camera_matrix_left);
     println!("Right Camera Matrix: {:?}", camera_matrix_right);
     println!("Rotation: {:?}", rotation);
     println!("Translation: {:?}", translation);
 
     Ok((rotation, translation, essential_matrix, fundamental_matrix))
 }

// Stereo rectification
fn stereo_rectify(
    rotation: &Mat, 
    translation: &Mat, 
    camera_size: Size,
    right_img: Mat,
    left_img: Mat, 
    roi1: &mut Rect, 
    roi2: &mut Rect) -> Result<(Mat, Mat, Mat, Mat, Mat), Error> {
    
    let mut r1 = Mat::default();
    let mut r2 = Mat::default();
    let mut p1 = Mat::default();
    let mut p2 = Mat::default();
    let mut q = Mat::default();
    let alpha: f64 = 0.0;
    let flag = 0;
    
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

    // // Rotation and translation between cameras
    // let rotation = Mat::from_slice_2d(&[
    //     [1.0, 0.0, 0.0],
    //     [0.0, 1.0, 0.0],
    //     [0.0, 0.0, 1.0],
    // ])?;
    // let translation = Mat::from_slice(&[0.1, 0.0, 0.0])?;

    
    
    
        opencv::calib3d::stereo_rectify(
        &camera_matrix_left,
        &dist_coeffs_left,
        &camera_matrix_right,
        &dist_coeffs_right,
        camera_size,
        &rotation,
        &translation,
        &mut r1,
        &mut r2,
        &mut p1,
        &mut p2,
        &mut q,
        flag,
        alpha,
        camera_size,
        roi1,
        roi2
    )?;
    
    let left_camera_size = camera_size.clone();
    let right_camera_size = camera_size.clone();

    let mut map1_left = Mat::default();
    let mut map2_left = Mat::default();
    let mut map1_right = Mat::default();
    let mut map2_right = Mat::default();

    calib3d::init_undistort_rectify_map(
        &camera_matrix_left,
        &dist_coeffs_left,
        &r1,
        &p1,
        left_camera_size,
        opencv::core::CV_32FC1,
        &mut map1_left,
        &mut map2_left,
    )?;

    calib3d::init_undistort_rectify_map(
        &camera_matrix_right,
        &dist_coeffs_right,
        &r2,
        &p2,
        right_camera_size,
        opencv::core::CV_32FC1,
        &mut map1_right,
        &mut map2_right,
    )?;

    let mut rectified_left = Mat::default();
    let mut rectified_right = Mat::default();

    imgproc::remap(
        &left_img,
        &mut rectified_left,
        &map1_left,
        &map2_left,
        imgproc::INTER_LINEAR,
        opencv::core::BORDER_CONSTANT,
        opencv::core::Scalar::default(),
    )?;

    imgproc::remap(
        &right_img,
        &mut rectified_right,
        &map1_right,
        &map2_right,
        imgproc::INTER_LINEAR,
        opencv::core::BORDER_CONSTANT,
        opencv::core::Scalar::default(),
    )?;

    Ok((r1, r2, p1, p2, q))
}

// Generate point cloud from disparity map
fn generate_point_cloud(q: &Mat, disparity: &Mat) -> Result<Mat, Error> {
    let mut points   = Mat::default();
    let ddepth = -1;
    opencv::calib3d::reproject_image_to_3d(
        &disparity,
        &mut points, 
        &q, 
        true,
        ddepth
    )?;
    
    Ok(points)
}

// Main function
pub fn calibrate_and_rectify(image: &Mat) -> Result<(), Error> {
    
    // Stereo calibration
    let (rotation, translation, essential_matrix, fundamental_matrix)  = stereo_calibrate()?;

    

    let (mut left_roi, mut right_roi) = establish_rois_zone(&image);
    let (camera_matrix_left, camera_matrix_right) = call_roi(
        &image, 
        &left_roi, 
        &right_roi)?;
    let new_img_size = camera_matrix_left.size()?;
    // Stereo rectification
    let (r1, r2, p1, p2, q)   = stereo_rectify( 
        &rotation, 
        &translation, 
        new_img_size,
        camera_matrix_right,
        camera_matrix_left,
        &mut left_roi, 
        &mut right_roi)?;
    
    let mut stereo_alg = opencv::calib3d::StereoSGBM::create(
        0,
        16, 
        3, 
        0, 
        0, 
        0, 
        0, 
        0, 
        0, 
        0, 
        StereoSGBM_MODE_SGBM)?;
        let mut disparity = Mat::default();


    stereo_alg.compute(&image, &p2, &mut disparity)?;
    let point_cloud = generate_point_cloud(&q, &disparity)?;
    // Normalize disparity for visualization
    let mut disparity_normalized = Mat::default();
    opencv::core::normalize(
        &disparity,
        &mut disparity_normalized,
        0.0,
        255.0,
        NORM_MINMAX,
        CV_16F,
        &Mat::default(),
    )?;

    // Now you can use the rectified parameters to process live video
    

    Ok(())
}