use opencv::prelude::*;
use opencv::{highgui, videoio, Result};
mod calibration;

use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std::{env, sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;
use sensor_msgs::msg::Image as ImageMsg;
use rclrs::msg::image::Image;




struct CameraDriverNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<ImageMsg>>,
}
impl CameraDriverNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "camera_node").unwrap();
        let publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, publisher })
    }
    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self.publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: i32 = 0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        count = publisher_other_thread.publish_data(count).unwrap();
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