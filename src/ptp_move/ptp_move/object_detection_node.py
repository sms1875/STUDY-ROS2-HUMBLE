# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import torch
# import warnings
# warnings.filterwarnings("ignore", category=FutureWarning)

# class ObjectDetectionNode(Node):
#     def __init__(self):
#         super().__init__('object_detection_node')

#         # ROS2 Publishers
#         self.image_pub = self.create_publisher(Image, 'detection/image', 10)
#         self.detection_pub = self.create_publisher(String, 'detection/objects', 10)

#         # YOLOv5 모델 로드
#         self.get_logger().info("Loading YOLOv5 model...")
#         self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
#         self.model.conf = 0.5  # Confidence threshold

#         # RealSense 카메라 초기화
#         self.pipeline = rs.pipeline()
#         self.config = rs.config()
#         self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         self.pipeline.start(self.config)

#         # ROS2 Timer
#         self.timer = self.create_timer(0.01, self.timer_callback)

#     def timer_callback(self):
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()

#         if not color_frame:
#             return

#         # 이미지 가져오기 및 YOLOv5 추론
#         color_image = np.asanyarray(color_frame.get_data())
#         results = self.model(color_image)

#         # 결과 이미지 생성
#         detection_image = np.array(results.render()[0])
#         detected_objects = results.pandas().xyxy[0].to_dict(orient="records")
        
#         # ROS 메시지로 변환 및 퍼블리시
#         detection_msg = String()
#         detection_msg.data = str(detected_objects)
#         self.detection_pub.publish(detection_msg)

#         # OpenCV 이미지를 ROS Image 메시지로 변환 후 퍼블리시
#         ros_image_msg = self.cv2_to_imgmsg(detection_image)
#         self.image_pub.publish(ros_image_msg)

#     def cv2_to_imgmsg(self, cv_image):
#         msg = Image()
#         msg.height, msg.width, _ = cv_image.shape
#         msg.encoding = 'bgr8'
#         msg.data = cv_image.tobytes()
#         msg.step = len(cv_image[0]) * 3
#         return msg

#     def destroy_node(self):
#         self.pipeline.stop()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import time

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # ROS2 Publishers
        self.image_pub = self.create_publisher(Image, 'detection/image', 10)
        self.detection_pub = self.create_publisher(String, 'detection/objects', 10)

        # YOLOv5 모델 로드 (GPU 사용 확인)
        self.get_logger().info("Loading YOLOv5 model...")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True).to(self.device)
        self.model.conf = 0.5  # Confidence threshold
        self.get_logger().info(f"Model loaded on {self.device}")

        # RealSense 카메라 초기화
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)  # FPS 조정
        self.pipeline.start(self.config)

        # ROS2 Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        start_time = time.time()

        # RealSense 이미지 캡처
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warning("No frame received from RealSense")
            return

        color_image = np.asanyarray(color_frame.get_data())

        # 이미지 크기 조정 (속도 개선)
        resized_image = cv2.resize(color_image, (320, 320))  # YOLO 입력 크기로 축소
        # input_image = resized_image[:, :, ::-1]  # BGR to RGB

        # YOLOv5 추론 (GPU 사용)
        results = self.model(resized_image)

        # 결과 처리
        detection_image = np.array(results.render()[0])
        detected_objects = results.pandas().xyxy[0].to_dict(orient="records")

        # ROS 메시지 퍼블리싱
        detection_msg = String()
        detection_msg.data = str(detected_objects)
        self.detection_pub.publish(detection_msg)

        ros_image_msg = self.cv2_to_imgmsg(detection_image)
        self.image_pub.publish(ros_image_msg)

        # 처리 속도 로깅
        fps = 1.0 / (time.time() - start_time)
        self.get_logger().info(f"Processed frame at {fps:.2f} FPS")

    def cv2_to_imgmsg(self, cv_image):
        msg = Image()
        msg.height, msg.width, _ = cv_image.shape
        msg.encoding = 'bgr8'
        msg.data = cv_image.tobytes()
        msg.step = len(cv_image[0]) * 3
        return msg

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
