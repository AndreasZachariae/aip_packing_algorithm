import rclpy
from rclpy.node import Node
from aip_packing_planning_interfaces.srv import PackItems

class PackItemsService(Node):

    def __init__(self):
        super().__init__('pack_items_service')
        self.srv = self.create_service(PackItems, 'items_to_pack', self.pack_items_callback)
        self.get_logger().info('Service server is ready.')

    def pack_items_callback(self, request, response):
        #     items = ["Box_Gluehlampe", "Box_Wischblatt", "Keilriemen_gross", "Box_Bremsbacke", "Keilriemen_klein", "Tuete"]

        # response.itemstopack = 'Item1, Item2, Item3'
        response.itemstopack = 'Box_Gluehlampe,Box_Wischblatt,Keilriemen_gross,Box_Bremsbacke,Keilriemen_klein,Tuete'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PackItemsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


##
#ros2 service call items_to_pack aip_packing_planning_interfaces/srv/PackItems '{}'