use opencv::prelude::*;
use opencv::{videoio, Result};
mod calibration;

use rclrs::{create_node, 
    Context, 
    Node, 
    Publisher, 
    RclrsError, 
    QOS_PROFILE_DEFAULT,
    ToLogParams};
use std::{env, sync::Arc, thread, time::Duration};
use sensor_msgs::msg::Image as ImageMsg;
use sensor_msgs::msg::CameraInfo as CameraInfoMsg;
use rclrs::log;
use builtin_interfaces::msg::Time;


struct CameraDriverNode {
    node: Arc<Node>,
    left_publisher: Arc<Publisher<ImageMsg>>,
    right_publisher: Arc<Publisher<ImageMsg>>,
    left_info_publisher: Arc<Publisher<CameraInfoMsg>>,
    right_info_publisher: Arc<Publisher<CameraInfoMsg>>,
}
impl CameraDriverNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "camera_node").unwrap();
        let left_publisher = node
            .create_publisher("left_image", QOS_PROFILE_DEFAULT)
            .unwrap();
        let right_publisher = node
            .create_publisher("right_image", QOS_PROFILE_DEFAULT)
            .unwrap();
        let left_info_publisher = node
            .create_publisher("left_camera_info", QOS_PROFILE_DEFAULT)
            .unwrap();
        let right_info_publisher = node
            .create_publisher("right_camera_info", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, left_publisher, right_publisher, left_info_publisher, right_info_publisher })
    }
    fn publish_info(&self, left_image: &Mat, right_image: &Mat) -> Result<(), RclrsError> {
        let mut left_msg: ImageMsg = ImageMsg::default();
        let mut right_msg: ImageMsg = ImageMsg::default();
		let now = self.node.get_clock().now();

        left_msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		left_msg.height = left_image.rows() as u32;
		left_msg.width = left_image.cols() as u32;
		left_msg.encoding = "bgr8".to_string();
		left_msg.is_bigendian = 0;
		left_msg.step = (left_image.cols() * 3) as u32;
		left_msg.data = left_image.data_bytes().unwrap().to_vec();
        self.left_publisher.publish(left_msg).unwrap();

		
        right_msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		right_msg.height = right_image.rows() as u32;
		right_msg.width = right_image.cols() as u32;
		right_msg.encoding = "bgr8".to_string();
		right_msg.is_bigendian = 0;
		right_msg.step = (right_image.cols() * 3) as u32;
		right_msg.data = right_image.data_bytes().unwrap().to_vec();
        self.right_publisher.publish(right_msg).unwrap();
        Ok(())
    }

 fn publish_data(&self, left_image: Mat, right_image: Mat) -> Result<Mat, RclrsError> {
        let mut left_msg: ImageMsg = ImageMsg::default();
        let mut right_msg: ImageMsg = ImageMsg::default();
		let now = self.node.get_clock().now();

        left_msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		left_msg.header.frame_id = "camera_frame".to_string();
		left_msg.height = left_image.rows() as u32;
		left_msg.width = left_image.cols() as u32;
		left_msg.encoding = "bgr8".to_string();
		left_msg.is_bigendian = 0;
		left_msg.step = (left_image.cols() * 3) as u32;
		left_msg.data = left_image.data_bytes().unwrap().to_vec();
        self.left_publisher.publish(left_msg).unwrap();

		
        right_msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		right_msg.header.frame_id = "camera_frame".to_string();
		right_msg.height = right_image.rows() as u32;
		right_msg.width = right_image.cols() as u32;
		right_msg.encoding = "bgr8".to_string();
		right_msg.is_bigendian = 0;
		right_msg.step = (right_image.cols() * 3) as u32;
		right_msg.data = right_image.data_bytes().unwrap().to_vec();
        self.right_publisher.publish(right_msg).unwrap();
        Ok(left_image)
    }
}
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let camera_node = Arc::new(CameraDriverNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&camera_node);
	let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY).unwrap(); // 0 is the default camera
	let opened = videoio::VideoCapture::is_opened(&cam).unwrap();
	if !opened {
		panic!("Unable to open default camera!");
	}
    let mut calib_frame: Mat = Mat::default();
    cam.read(&mut calib_frame).unwrap();
    let (left_roi, right_roi) = calibration::establish_rois_zone(&calib_frame);

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(33));
 		let mut frame = Mat::default();
		cam.read(&mut frame).unwrap();
        opencv::core::flip(&mut frame.clone(), &mut frame, 0).unwrap();
        let left_frame = Mat::roi(&mut frame, left_roi).unwrap().clone_pointee();
        let right_frame = Mat::roi(&mut frame, right_roi).unwrap().clone_pointee();
        publisher_other_thread.publish_info(&left_frame, &right_frame).unwrap();
		publisher_other_thread.publish_data(left_frame, right_frame).unwrap();
        
    });
    rclrs::spin(camera_node.node.clone())
}






//camera calibration works in ros2, I did not need to rewrite that wheel.
//Tasks ahead of me:
//2. add some yolo object detection
//3. add some point cloud stuff
//4. profit
//actual 4, document code and make it a package
//5. see  if this runs on the nano