import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose  # 구독할 topic의 type


class TurtlesimSubscriber(Node):  # Node 상속
    def __init__(self):
        super().__init__("turtlesim_subscriber")  # Node의 init
        self.subscription = self.create_subscription(  # subscriptions 생성
            Pose,  # type
            "/turtle1/pose",  # topic
            self.callback,  # callback
            10,  # 데이터 queue 크기
        )

    def callback(self, msg):
        print("X: ", msg.x, ", Y: ", msg.y)  # 좌표 출력


def main():
    rp.init()

    turtlesim_subscriber = TurtlesimSubscriber()
    rp.spin(turtlesim_subscriber)

    turtlesim_subscriber.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()
