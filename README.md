# Corn YOLO ROS Detection Package

This package provides a ROS 2 node for real-time object detection using YOLO, designed for dual-camera setups. It is ready to use with Docker for easy deployment.

## Features
- Runs YOLO detection on two camera streams
- Publishes detection results as ROS 2 messages
- Configurable via parameters or YAML config
- Dockerfile for reproducible environment

## Quick Start with Docker

### 1. Build the Docker Image
```bash
docker build -t corn_yolo_detection .
```

### 2. Run the Container
```bash
docker run --rm -it \
  --network host \
  corn_yolo_detection
```
- Adjust `--device` flags to match your camera devices.
- The container will launch the detection node by default.

### 3. Custom Model or Parameters
- To use a custom model, mount your weights and override the parameter:
```bash
docker run --rm -it \
  --network host \
  -v /path/to/your/model.pt:/ros2_ws/model/best.pt \
  corn_yolo_detection \
  ros2 run corn_yolo_ros_interface detection.py --ros-args -p model_path:=/ros2_ws/model/best.pt
```
- To use a custom config file, mount and override:
```bash
docker run --rm -it \
  --network host \
  -v /path/to/config.yaml:/ros2_ws/src/corn_yolo_ros_interface/config/config.yaml \
  corn_yolo_detection
```

## ROS 2 Topics
- **Input:**
  - `/cam1/pylon_ros2_camera_node/image_raw` (sensor_msgs/Image)
  - `/cam2/pylon_ros2_camera_node/image_raw` (sensor_msgs/Image)
- **Output:**
  - `detections1` (corn_yolo_ros_interface/DetectionArray)
  - `detections2` (corn_yolo_ros_interface/DetectionArray)

## Parameters
Parameters can be set via command line or YAML config (see `config/config.yaml`). Key parameters:
- `cam1.topic`, `cam2.topic`: Input image topics
- `model_path`: Path to YOLO weights
- `cam1.frame`, `cam2.frame`: Frame names for detections

## Example Launch (ROS 2)
You can also launch using ROS 2 launch files:
```bash
ros2 launch corn_yolo_ros_interface launch_detection.py
```

## Requirements
- ROS 2 Jazzy (or compatible)
- Python dependencies (see `requirements.txt`)
- Camera devices accessible in the container

---
For more details, see the source code and configuration files in this package.
