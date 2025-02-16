use opencv::prelude::*;
use opencv::{videoio, Result};
mod calibration;

use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std::{env, sync::Arc, thread, time::Duration};
use sensor_msgs::msg::Image as ImageMsg;

use builtin_interfaces::msg::Time;


struct CameraDriverNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<ImageMsg>>,
}
impl CameraDriverNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "camera_node").unwrap();
        let publisher = node
            .create_publisher("stereo_image", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, publisher })
    }
    fn publish_data(&self, image: Mat) -> Result<Mat, RclrsError> {
        let mut msg: ImageMsg = ImageMsg::default();
		let now = self.node.get_clock().now();
        msg.header.stamp =  Time {
            sec: (now.nsec / 1000000000) as i32,
            nanosec: now.nsec as u32,
        };
		msg.header.frame_id = "camera_frame".to_string();
		msg.height = image.rows() as u32;
		msg.width = image.cols() as u32;
		msg.encoding = "bgr8".to_string();
		msg.is_bigendian = 0;
		msg.step = (image.cols() * 3) as u32;
		msg.data = image.data_bytes().unwrap().to_vec();
        self.publisher.publish(msg).unwrap();
        Ok(image)
    }
}
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let publisher = Arc::new(CameraDriverNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
	let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY).unwrap(); // 0 is the default camera
	let opened = videoio::VideoCapture::is_opened(&cam).unwrap();
	if !opened {
		panic!("Unable to open default camera!");
	}


    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(33));
 		let mut frame = Mat::default();
		cam.read(&mut frame).unwrap();
		frame = publisher_other_thread.publish_data(frame).unwrap();
    });
    rclrs::spin(publisher.node.clone())
}




// fn main() -> Result<()> {
// 	let window = "video capture";
// 	highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;
// 	let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
// 	let opened = videoio::VideoCapture::is_opened(&cam)?;
// 	if !opened {
// 		panic!("Unable to open default camera!");
// 	}
// 	loop {
// 		let mut frame = Mat::default();
// 		cam.read(&mut frame)?;
// 		if frame.size()?.width > 0 {
// 			calibration::calibrate_and_rectify(&frame)?;
// 			highgui::imshow(window, &frame)?;
// 		}
// 		let key = highgui::wait_key(10)?;
// 		if key > 0 && key != 255 {
// 			break;
// 		}
// 	}
// 	Ok(())
// }

//camera calibration works in ros2, I did not need to rewrite that wheel.
//Tasks ahead of me:
//1. impose this publisher to publish both images
//2. add some yolo object detection
//3. add some point cloud stuff
//4. profit
//actual 4, document code and make it a package
//5. see  if this runs on the nano