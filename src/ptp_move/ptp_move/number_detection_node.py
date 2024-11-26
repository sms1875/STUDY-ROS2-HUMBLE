import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
from cv_bridge import CvBridge


class NumberDetection:
    """이미지에서 숫자를 인식하는 클래스"""
    def __init__(self):
        # RealSense 카메라 초기화
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # YOLOv5 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.5  # Confidence threshold

    def detect_numbers(self):
        # RealSense 카메라에서 이미지 가져오기
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())

        # YOLOv5를 사용하여 객체 감지
        results = self.model(color_image)

        # 감지된 이미지 시각화
        detection_image = np.array(results.render()[0])

        # 숫자 감지 처리
        detected_numbers = []
        for result in results.pandas().xyxy[0].itertuples():
            if result.name.isdigit():  # 숫자만 필터링
                detected_numbers.append(result.name)

        return detected_numbers, detection_image

    def stop_camera(self):
        self.pipeline.stop()


class NumberDetectionNode(Node):
    """ROS2 노드: 숫자를 인식하고 결과를 퍼블리시"""
    def __init__(self):
        super().__init__('number_detection_node')

        # 숫자 인식 클래스 초기화
        self.number_detection = NumberDetection()

        # ROS2 Publisher
        self.image_pub = self.create_publisher(Image, 'detection/image', 10)
        self.detection_pub = self.create_publisher(String, 'detection/numbers', 10)

        # CvBridge 초기화
        self.bridge = CvBridge()

        # 타이머 설정
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        detected_numbers, detection_image = self.number_detection.detect_numbers()
        if detected_numbers is not None:
            # 감지된 숫자 로깅 및 퍼블리시
            self.get_logger().info(f"Detected numbers: {detected_numbers}")
            detection_msg = String()
            detection_msg.data = ','.join(detected_numbers)
            self.detection_pub.publish(detection_msg)

            # 감지된 이미지 퍼블리시
            if detection_image is not None:
                ros_image_msg = self.bridge.cv2_to_imgmsg(detection_image, encoding="bgr8")
                self.image_pub.publish(ros_image_msg)

    def destroy_node(self):
        self.number_detection.stop_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NumberDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
