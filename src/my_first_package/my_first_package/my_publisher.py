import rclpy as rp
from rclpy.node import Node

from geometry_msgs.msg import Twist


class TurtlesimPublisher(Node):  # Node 상속
    def __init__(self):
        super().__init__("turtlesim_publisher")  # Node의 init
        self.publisher = self.create_publisher(  # publisher 생성
            Twist,  # type
            "/turtle1/cmd_vel",  # topic
            10,  # 데이터 queue 크기
        )
        timer_period = 0.5  # 500ms 마다 발행
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publisher.publish(msg)


def main(args=None):
    rp.init(args=args)

    turtlesim_publisher = TurtlesimPublisher()
    rp.spin(turtlesim_publisher)

    turtlesim_publisher.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()
