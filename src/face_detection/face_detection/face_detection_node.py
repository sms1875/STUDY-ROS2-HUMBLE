import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class FaceDetectionNode(Node):
    def __init__(self):
        # 노드 초기화
        super().__init__('face_detection_node')
        
        # 이미지 토픽 구독자 생성 (카메라 이미지 데이터 구독)
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # 얼굴 감지 데이터를 퍼블리시할 토픽 생성
        self.publisher = self.create_publisher(String, '/face_detection_data', 10)
        
        # CvBridge 객체 생성 (ROS 이미지를 OpenCV 이미지로 변환)
        self.bridge = CvBridge()
        
        # 얼굴 인식을 위한 Haar Cascade 분류기 로드
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 그레이스케일 이미지로 변환 (얼굴 감지에 사용)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 얼굴 감지 수행
        faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # 감지된 얼굴 각각에 대해
        for (x, y, w, h) in faces:
            # 얼굴 주위에 사각형(바운딩 박스) 그리기
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)

            # 얼굴 감지 데이터 퍼블리시
            face_msg = String()
            face_msg.data = f"Face detected at x: {x}, y: {y}, width: {w}, height: {h}"
            self.publisher.publish(face_msg)

            # 얼굴 감지 위치 로그 출력
            self.get_logger().info(f"Face detected at x: {x}, y: {y}, width: {w}, height: {h}")

        # 감지된 결과가 표시된 프레임을 화면에 출력
        cv2.imshow('Face Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    # rclpy 초기화
    rclpy.init(args=args)
    
    # FaceDetectionNode 인스턴스 생성 및 실행
    node = FaceDetectionNode()
    rclpy.spin(node)
    
    # 노드 종료 및 rclpy 종료
    node.destroy_node()
    rclpy.shutdown()

# 메인 함수 실행
if __name__ == '__main__':
    main()
