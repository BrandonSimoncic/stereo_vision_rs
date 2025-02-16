# Stereo Vision ROS2 Rust Node

This ROS2 Rust node is designed for stereo vision applications. It processes images from a pair of cameras to perform tasks such as depth estimation, disparity map generation, and 3D reconstruction. The node subscribes to image topics published by the stereo camera setup, processes the images using computer vision algorithms, and publishes the results to relevant topics.

## Key Functionalities
- Subscribes to left and right image topics from stereo cameras.
- Performs stereo image processing to compute depth information.
- Publishes the computed depth or disparity maps to output topics.

## Usage
1. Ensure that the ROS2 environment is properly set up.
2. Launch the node along with the stereo camera drivers.
3. Configure the input and output topics as needed.

This node leverages the power of Rust for performance and safety, making it suitable for real-time stereo vision applications in robotics and automation.
