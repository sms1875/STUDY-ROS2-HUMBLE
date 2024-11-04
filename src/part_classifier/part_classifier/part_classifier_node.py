import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 카테고리 매핑
categories = {
    'battery': '전원 부품',
    'charger': '전원 부품',
    'brush': '청소 부품',
    'filter': '청소 부품',
    'sensor': '감지 부품',
    'bumper': '감지 부품',
    'motor': '운동 부품',
    'wheel': '운동 부품'
}

# 토픽 이름
SUBSCRIBER_TOPIC = '/robot_part'
PUBLISHER_TOPIC = 'part_category'

class PartClassifierNode(Node):
    def __init__(self):
        # 노드 초기화
        super().__init__('part_classifier_node')
        
        # 부품 이름을 구독하는 Subscriber 생성
        self.subscription = self.create_subscription(String, SUBSCRIBER_TOPIC, self.part_callback, 10)
        
        # 부품 카테고리 결과를 퍼블리시할 Publisher 생성
        self.publisher = self.create_publisher(String, PUBLISHER_TOPIC, 10)
        
        # 부품 이름과 카테고리 매핑
        self.categories = categories

    def part_callback(self, msg):
        # 수신된 부품 이름에서 카테고리 분류
        part_name = msg.data
        category = self.categories.get(part_name)  # 카테고리 매칭
        print(category)
        # 카테고리 결과를 String 메시지로 생성
        category_msg = String()
        category_msg.data = category

        # 카테고리 결과 퍼블리시
        self.publisher.publish(category_msg)
        self.get_logger().info(f"Part: {part_name}, Category: {category}")

def main(args=None):
    # rclpy 초기화
    rclpy.init(args=args)
    
    # PartClassifierNode 인스턴스 생성 및 실행
    node = PartClassifierNode()
    rclpy.spin(node)
    
    # 노드 종료 및 rclpy 종료
    node.destroy_node()
    rclpy.shutdown()

# 메인 함수 실행
if __name__ == '__main__':
    main()
