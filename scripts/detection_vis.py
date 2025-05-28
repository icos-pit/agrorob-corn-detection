#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class Detector(Node):
    def __init__(self):
        super().__init__('detector')

        # Parameters
        self.declare_parameter('cam1.topic', '/image_raw')
        self.declare_parameter('cam1.info_topic', '/camera_info')
        self.declare_parameter('cam1.frame', 'cam1')
        self.declare_parameter('model_path', '/corn_ws/model/weights/best.pt')

        # Get parameters
        self.cam1_topic = self.get_parameter('cam1.topic').value
        self.cam1_info_topic = self.get_parameter('cam1.info_topic').value

        self.get_logger().info(f"cam_topic: {self.cam1_topic}")

        self.cam1_info = get_default_camera_info()

        # Load model
        self.model1 = YOLO(self.get_parameter('model_path').value)
        self.model1.model.names = {0: "weeds", 1: "corn"}

        if self.model1 is None:
            self.get_logger().error("Could not load model from path")
            return
        
        # Publishers
        self.detection_pub = self.create_publisher(Image, 'detection', 10)

        # Subscribers
        self.bridge = CvBridge()
        self.cam1_sub = self.create_subscription(Image, self.cam1_topic, lambda msg: self.image_callback(msg, self.model1), 1)
    
    def image_callback(self, msg, model=None):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # print(f"Image shape: {img.shape}")
        # results = model.track(img, persist=True, show=True, conf=0.5)
        results = model.predict(img, conf=0.5, show=True)
        return

        

        # print('-'*50)
        # print(results)
        # print('-'*50)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = list(box.xyxy.reshape(-1).tolist())
                conf = box.conf
                cls = box.cls
                # print(f"x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}, conf: {conf}, cls: {cls}")
                if cls == 1 and conf >= 0.5:
                    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        self.detection_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))


def get_default_camera_info() -> CameraInfo:
    def_info =  {
        'width': 1600,
        'height': 1200,
        'distortion_model': 'plumb_bob',
        'D': [0.0, 0.0, 0.0, 0.0, 0.0],
        'K': [1000.0, 0.0, 800.0,  # fx, 0, cx
            0.0, 1000.0, 600.0,  # 0, fy, cy
            0.0,  0.0,   1.0],
        'R': [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ],
        'P': [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ],
        'binning_x': 0,
        'binning_y': 0,
        'roi': {
            'x_offset': 0,
            'y_offset': 0,
            'height': 0,
            'width': 0,
            'do_rectify': False
        }
    }

    info = CameraInfo()
    info.width = def_info['width']
    info.height = def_info['height']
    info.distortion_model = def_info['distortion_model']
    info.d = def_info['D']
    info.k = def_info['K']
    info.r = def_info['R']
    info.p = def_info['P']
    info.binning_x = def_info['binning_x']
    info.binning_y = def_info['binning_y']
    info.roi.x_offset = def_info['roi']['x_offset']
    info.roi.y_offset = def_info['roi']['y_offset']
    info.roi.height = def_info['roi']['height']
    info.roi.width = def_info['roi']['width']
    info.roi.do_rectify = def_info['roi']['do_rectify']
    return info



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
