import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class QRCodeDetectionNode(Node):
    def __init__(self):
        # 노드 초기화
        super().__init__('qr_code_detection_node')
        
        # 카메라 이미지 데이터를 구독하는 구독자 생성
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # QR 코드 데이터를 퍼블리시할 퍼블리셔 생성
        self.publisher = self.create_publisher(String, '/qr_code_data', 10)
        
        # CvBridge 객체 생성 (ROS 이미지를 OpenCV 이미지로 변환)
        self.bridge = CvBridge()
        
        # QR 코드 검출을 위한 OpenCV QRCodeDetector 생성
        self.qr_code_detector = cv2.QRCodeDetector()

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # QR 코드 검출 및 디코딩
        data, points, _ = self.qr_code_detector.detectAndDecode(frame)
        
        # QR 코드가 감지되었을 때
        if points is not None and data:
            # QR 코드 주위에 사각형(바운딩 박스) 그리기
            points = points[0].astype(int)
            for i in range(len(points)):
                pt1 = tuple(points[i])
                pt2 = tuple(points[(i + 1) % len(points)])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 3)

            # 이미지 위에 QR 코드 데이터 텍스트로 표시
            cv2.putText(frame, data, (points[0][0], points[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # QR 코드 데이터를 퍼블리시
            qr_msg = String()
            qr_msg.data = data
            self.publisher.publish(qr_msg)

            # 감지된 QR 코드 데이터 로그에 출력
            self.get_logger().info(f"QR Code detected : {data}")

        # 주석 및 바운딩 박스가 포함된 프레임을 화면에 출력
        cv2.imshow('QR Code Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    # rclpy 초기화
    rclpy.init(args=args)
    
    # QRCodeDetectionNode 인스턴스 생성 및 실행
    node = QRCodeDetectionNode()
    rclpy.spin(node)
    
    # 노드 종료 및 rclpy 종료
    node.destroy_node()
    rclpy.shutdown()

# 메인 함수 실행
if __name__ == '__main__':
    main()
