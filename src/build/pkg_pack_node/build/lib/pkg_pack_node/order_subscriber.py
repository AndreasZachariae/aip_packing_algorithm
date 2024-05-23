import rclpy
from rclpy.node import Node
#from std_msgs.msg import list
from std_msgs.msg import String

#from Pack_Algorithm import Order_class
from Classes.topic_data import Order_class


class OrderSubscriber(Node):
    def __init__(self):
        super().__init__('order_subscriber')

        print('Order Subscriber Node started')

        self.subscription = self.create_subscription(
            String,
            '/order_list',
            self.listener_callback,
            10)
        
        print('Subscription created')

        self.subscription  # verhindern, dass die Variable gelöscht wird

    def listener_callback(self, msg):
        self.get_logger().info(f'------------------------------------------\n')

        self.get_logger().info(f'Received Order: {msg.data}')
        Order_class.set_topic_data(msg.data)
        print(Order_class.get_topic_data())
        from Pack_Algorithm import Pack_Algorithm
        Pack_Algorithm.generate_output()



def main(args=None):
    rclpy.init(args=args)
    order_subscriber = OrderSubscriber()
    rclpy.spin(order_subscriber)
    order_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
