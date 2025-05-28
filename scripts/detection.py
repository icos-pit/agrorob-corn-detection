#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from corn_detections_msgs.msg import DetectionArray, Detection
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

def get_default_camera_info() -> CameraInfo:
    info = CameraInfo()
    info.width = 1600
    info.height = 1200
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5
    info.k = [1000.0, 0.0, 800.0, 0.0, 1000.0, 600.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.binning_x = 0
    info.binning_y = 0
    info.roi.x_offset = 0
    info.roi.y_offset = 0
    info.roi.height = 0
    info.roi.width = 0
    info.roi.do_rectify = False
    return info

class Detector(Node):
    def __init__(self):
        super().__init__('detector')

        # Declare and get parameters
        self.declare_parameter('cam1.topic', '/cam1/pylon_ros2_camera_node/image_raw')
        self.declare_parameter('cam1.info_topic', '/cam1/pylon_ros2_camera_node/camera_info')
        self.declare_parameter('cam2.topic', '/cam2/pylon_ros2_camera_node/image_raw')
        self.declare_parameter('cam2.info_topic', '/cam2/pylon_ros2_camera_node/camera_info')
        self.declare_parameter('cam1.frame', 'cam1')
        self.declare_parameter('cam2.frame', 'cam2')
        self.declare_parameter('model_path', '/corn_ws/src/corn_yolo_ros_interface/detection/model/best.pt')

        self.cam1_topic = self.get_parameter('cam1.topic').value
        self.cam1_info_topic = self.get_parameter('cam1.info_topic').value
        self.cam2_topic = self.get_parameter('cam2.topic').value
        self.cam2_info_topic = self.get_parameter('cam2.info_topic').value
        self.cam1_frame = self.get_parameter('cam1.frame').value
        self.cam2_frame = self.get_parameter('cam2.frame').value
        self.model_path = self.get_parameter('model_path').value

        self.get_logger().info(f"cam1_topic: {self.cam1_topic}")
        self.get_logger().info(f"cam2_topic: {self.cam2_topic}")

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera info
        self.cam1_info = get_default_camera_info()
        self.cam2_info = get_default_camera_info()

        # Load models
        self.model1 = YOLO(self.model_path)
        self.model2 = YOLO(self.model_path)

        if self.model1 is None or self.model2 is None:
            self.get_logger().error("Could not load model from path")
            return

        # Publishers
        self.detection1_pub = self.create_publisher(DetectionArray, 'detections1', 10)
        self.detection2_pub = self.create_publisher(DetectionArray, 'detections2', 10)

        # Subscribers
        self.bridge = CvBridge()
        self.cam1_sub = self.create_subscription(
            Image, self.cam1_topic,
            lambda msg: self.image_callback(msg, self.cam1_frame, self.model1, self.detection1_pub), 0)
        self.cam2_sub = self.create_subscription(
            Image, self.cam2_topic,
            lambda msg: self.image_callback(msg, self.cam2_frame, self.model2, self.detection2_pub), 0)

    def image_callback(self, msg, camera_frame, model, publisher):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = DetectionArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = camera_frame

        results = model.track(img, persist=True, verbose=False)
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = list(box.xyxy.reshape(-1).tolist())
                conf = float(box.conf)
                cls = int(box.cls)
                if cls == 1 and conf >= 0.5:
                    detection = Detection()
                    detection.x = (x1 + x2) / 2
                    detection.y = (y1 + y2) / 2
                    detection.w = x2 - x1
                    detection.h = y2 - y1
                    detection.confidence = conf
                    detection.class_id = cls
                    if box.id is None:
                        detection.id = -1
                    else:
                        detection.id = int(box.id)
                    detections.detections.append(detection)

        if detections.detections:
            self.get_logger().info(f"Detected {len(detections.detections)} object" + ("s" if len(detections.detections) > 1 else "") + f" in {camera_frame}")
            publisher.publish(detections)

def main(args=None):
    rclpy.init(args=args)
    detector = Detector()
    detector.get_logger().info("Node created")
    detector.get_logger().info("Starting detection node")
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
