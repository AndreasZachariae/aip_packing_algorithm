import rclpy
from rclpy.node import Node
from std_msgs.msg import List

class OrderSubscriber(Node):
    def __init__(self):
        super().__init__('order_subscriber')
        self.subscription = self.create_subscription(
            List,
            'order_list',
            self.listener_callback,
            10)
        self.subscription  # verhindern, dass die Variable gelöscht wird

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Order: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    order_subscriber = OrderSubscriber()
    rclpy.spin(order_subscriber)
    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
