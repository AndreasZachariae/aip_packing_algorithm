import rclpy
from rclpy.node import Node
from std_msgs.msg import List
import random

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(List, 'order_list', 10)

    def timer_callback(self):
        msg = List()
        msg.data = #
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing order list: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    order_publisher = OrderPublisher()
    rclpy.spin(order_publisher)
    order_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()