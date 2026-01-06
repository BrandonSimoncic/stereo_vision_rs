use opencv::imgproc::get_rotation_matrix_2d;
use opencv::prelude::*;
use opencv::{videoio, Result};
mod calibration;

use rclrs::*;
use std::{sync::Arc, thread, time::Duration};
use sensor_msgs::msg::Image as ImageMsg;
use sensor_msgs::msg::CameraInfo as CameraInfoMsg;
use sensor_msgs::msg::RegionOfInterest as RegionOfInterestMsg;
use sensor_msgs::srv::SetCameraInfo;
use rclrs::log;
use builtin_interfaces::msg::Time;


struct CameraParams {
    camera_matrix: Mat,
    distortion_coeffs: Mat,
    rotation_matrix: Mat,
    projection_matrix: Mat,
}

struct CameraDriverNode {
    node: Arc<Node>,
    left_camera: CameraParams,
    right_camera: CameraParams,
    left_publisher: Arc<Publisher<ImageMsg>>,
    right_publisher: Arc<Publisher<ImageMsg>>,
    left_info_publisher: Arc<Publisher<CameraInfoMsg>>,
    right_info_publisher: Arc<Publisher<CameraInfoMsg>>,
}
impl CameraDriverNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("camera_node")?;
        let left_publisher = node
            .create_publisher("left_image")?;
        let right_publisher = node
            .create_publisher("right_image")?;
        let left_info_publisher = node
            .create_publisher("left_camera_info")?;
        let right_info_publisher = node
            .create_publisher("right_camera_info")?;

        let left_camera = CameraParams {
            camera_matrix: calibration::load_matrix("left_camera_matrix").unwrap(),
            distortion_coeffs: calibration::load_matrix("left_distortion_coeffs").unwrap(),
            rotation_matrix: calibration::load_matrix("left_rotation_matrix").unwrap(),
            projection_matrix: calibration::load_matrix("left_projection_matrix").unwrap(),
        };
        let right_camera = CameraParams {
            camera_matrix: calibration::load_matrix("right_camera_matrix").unwrap(),
            distortion_coeffs: calibration::load_matrix("right_distortion_coeffs").unwrap(),
            rotation_matrix: calibration::load_matrix("right_rotation_matrix").unwrap(),
            projection_matrix: calibration::load_matrix("right_projection_matrix").unwrap(),
        };
        // let camera_info_service = node.create_service::<SetCameraInfo, _>(
        //     "set_camera_info",
        //     |request, response| {
        //         // Handle the request and set the camera info
        //         response.camera_info = self.construct_infomsg(&Mat::default()).unwrap();
        //         response
        //     }
        // )?;
        Ok(Self { node: node.into(), left_publisher: left_publisher.into(), right_publisher: right_publisher.into(), left_info_publisher: left_info_publisher.into(), right_info_publisher: right_info_publisher.into(), left_camera, right_camera })
    }
    fn publish_info(&self, left_image: &Mat, right_image: &Mat) -> Result<(), RclrsError> {
        let left_msg = self.construct_infomsg(left_image).unwrap();
        let right_msg = self.construct_infomsg(right_image).unwrap();
		self.left_info_publisher.publish(left_msg).unwrap();
        self.right_info_publisher.publish(right_msg).unwrap();
        Ok(())
    }
    fn construct_infomsg(&self, image: &Mat) -> Result<CameraInfoMsg, RclrsError> {
        let mut msg: CameraInfoMsg = CameraInfoMsg::default();
        let now = self.node.get_clock().now();

        msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
        msg.header.frame_id = "camera_frame".to_string();
        msg.height = image.rows() as u32;
        msg.width = image.cols() as u32;
        msg.distortion_model = "plumb_bob".to_string();
        msg.d = Vec::<f64>::new();
        msg.k = calibration::mat_to_array(&self.left_camera.camera_matrix).unwrap();
        msg.r = calibration::mat_to_array(&self.left_camera.rotation_matrix).unwrap();
        msg.p = calibration::mat_to_array12(&self.left_camera.projection_matrix).unwrap();
        msg.binning_x = 0;
        msg.binning_y = 0;
        msg.roi.width = image.cols() as u32;
        msg.roi.height = image.rows() as u32;
        msg.roi.x_offset = 0;
        msg.roi.y_offset = 0;
        Ok(msg)

    }

    fn publish_data(&self, left_image: &Mat, right_image: &Mat) -> Result<(), RclrsError> {
        let left_msg = self.construct_imagemsg(left_image, "left_image".to_string()).unwrap();
        let right_msg = self.construct_imagemsg(right_image, "right_image".to_string()).unwrap();
		self.left_publisher.publish(left_msg).unwrap();
        self.right_publisher.publish(right_msg).unwrap();
        Ok(())
    }
    fn construct_imagemsg(&self, image: &Mat, frame: String) -> Result<ImageMsg, RclrsError> {
        let mut msg: ImageMsg = ImageMsg::default();
		let now = self.node.get_clock().now();
        msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		msg.header.frame_id = frame;
		msg.height = image.rows() as u32;
		msg.width = image.cols() as u32;
		msg.encoding = "bgr8".to_string();
		msg.is_bigendian = 0;
		msg.step = (image.cols() * 3) as u32;
		msg.data = image.data_bytes().unwrap().to_vec();
        Ok(msg)
    }


}

fn open_camera() -> videoio::VideoCapture {
    let cam = videoio::VideoCapture::new(0, videoio::CAP_ANY).unwrap(); // 0 is the default camera
    let opened = videoio::VideoCapture::is_opened(&cam).unwrap();
    if !opened {
        panic!("Unable to open default camera!");
    }
    cam
}


fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let camera_node = Arc::new(CameraDriverNode::new(&executor)?);
    let publisher_other_thread = Arc::clone(&camera_node);
    // let log_text = Arc::clone(&camera_node);
	let mut cam = open_camera();
    let mut calib_frame: Mat = Mat::default();
    
    cam.read(&mut calib_frame).unwrap();
    
    let (left_roi, right_roi) = calibration::establish_rois_zone(&calib_frame);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(33));

 		let mut frame = Mat::default();
		cam.read(&mut frame).unwrap();
        
        // log!(log_text.node.info(), "Publishing image");

        opencv::core::flip(&mut frame.clone(), &mut frame, 0).unwrap();

        let left_frame = Mat::roi(&mut frame, left_roi).unwrap().clone_pointee();
        let right_frame = Mat::roi(&mut frame, right_roi).unwrap().clone_pointee();
        
        // TODO: Fix this info stream to calibrate cameras
        // publisher_other_thread.publish_info(&left_frame, &right_frame).unwrap();
		publisher_other_thread.publish_data(&left_frame, &right_frame).unwrap();
        
    });
    executor.spin(SpinOptions::default()).first_error()
}
