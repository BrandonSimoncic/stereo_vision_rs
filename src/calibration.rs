
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

fn get_size(image: Mat) -> (i32, i32) 
{
    let rows = image.rows() as i32;
    let cols = image.cols() as i32;

    (rows, cols)
}


pub fn establish_rois_zone(scope: &Mat) -> (Rect, Rect)
{
    let scope = scope.clone();
    let (rows, cols) =  get_size(scope);
    let left_roi = Rect::new(0,0,cols / 2, rows);
    let right_roi = Rect::new(cols / 2, 0, cols - cols / 2, rows);
    (left_roi, right_roi)
}

#[allow(dead_code)]
fn call_roi(image: &Mat, left_roi:  &Rect, right_roi:  &Rect) -> Result<(Mat, Mat), Error> {
    let left = Mat::roi(image, *left_roi)?;
    let right = Mat::roi(image, *right_roi)?;
    let left = left.clone_pointee();
    let right = right.clone_pointee();

    Ok((left, right))
}



#[allow(dead_code)]
fn stereo_calibrate(left_image: &Mat, right_image: &Mat) -> Result<(Mat, Mat, Mat, Mat), Error> {
    // Chessboard dimensions (width x height of internal corners)
    let chessboard_size = Size::new(9, 6);

    // Square size of the chessboard in your desired unit (e.g., meters or millimeters)
    let square_size = 0.015; // 15 mm or 0.015 meters

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
 
     
    let left_img = left_image;
    let right_img = right_image;

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
    
    // Calibration outputs
    let mut camera_matrix_left =  Mat::default();
    let mut dist_coeffs_left = Mat::default();
    let mut camera_matrix_right = Mat::default();
    let mut dist_coeffs_right = Mat::default();


    calib3d::calibrate_camera(
        &object_points,
        &image_points_left,
        left_img.size()?,
        &mut camera_matrix_left,
        &mut dist_coeffs_left,
        &mut Mat::default(),
        &mut Mat::default(),
        0,
        TermCriteria::new(
            TermCriteria_COUNT + TermCriteria_EPS,
            100,
            1e-5,
        )?,
    )?;
    calib3d::calibrate_camera(
        &object_points,
        &image_points_right,
        right_img.size()?,
        &mut camera_matrix_right,
        &mut dist_coeffs_right,
        &mut Mat::default(),
        &mut Mat::default(),
        0,
        TermCriteria::new(
            TermCriteria_COUNT + TermCriteria_EPS,
            100,
            1e-5,
        )?,
    )?;
 

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
#[allow(unused_variables)]
fn stereo_rectify(
    camera_matrix_left: &Mat,
    dist_coeffs_left: &Mat,
    camera_matrix_right: &Mat,
    dist_coeffs_right: &Mat,
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

pub fn mat_to_array(mat: &Mat) -> Result<[f64; 9], opencv::Error> {
    // // Ensure the Mat has the correct number of elements
    // assert_eq!(mat.total(), 9);

    // Get the data as a slice
    let data: &[f64] = mat.data_typed()?;

    // Convert the slice to an array
    let array: [f64; 9] = data.try_into().expect("Slice with incorrect length");

    Ok(array)
}
pub fn mat_to_array12(mat: &Mat) -> Result<[f64; 12], opencv::Error> {
    // // Ensure the Mat has the correct number of elements
    // assert_eq!(mat.total(), 9);

    // Get the data as a slice
    let data: &[f64] = mat.data_typed()?;

    // Convert the slice to an array
    let array: [f64; 12] = data.try_into().expect("Slice with incorrect length");

    Ok(array)
}
 #[allow(dead_code)]
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
pub fn load_matrix(name: &str) -> Result<Mat, Error> {
    // let mut file = std::fs::File::open(name)?;
    // let mut buffer = Vec::new();
    // file.read_to_end(&mut buffer)?;
    // let mat = Mat::from_slice(&buffer)?;
    Ok(Mat::default())
}
// pub fn save_matrix(name: &str, mat: &Mat) -> Result<(), Error> {
//     let mut file = std::fs::File::create(name)?;
//     let buffer = mat.to_vec()?;
//     file.write_all(&buffer)?;
//     Ok(())
// }