import rclpy
from rclpy.node import Node
from aip_packing_planning_interfaces.srv import PackSequence
from Classes.items_transfer import items_transfer
from Classes.Packplan import Packplan


class PackItemsService(Node):

    def __init__(self):
        super().__init__('pack_items_service')
        self.srv = self.create_service(PackSequence, 'pack_planning', self.pack_items_callback)
        self.packing_client = self.create_client(PackSequence, 'pack_planning', )

        self.get_logger().info('Service server is ready.')

    def pack_items_callback(self, packing_request, response):
        
        items = []
        item_data = packing_request.objects_to_pick
        items = item_data.split(",")

        # Setze Request
        items_transfer.set_items(items)

        # Starte Packplanung
        from Pack_Algorithm import Pack_Algorithm
        # Pack_Algorithm()
    
        result = Packplan.get_packplan()
        

        return packing_request

def main(args=None):
    rclpy.init(args=args)
    node = PackItemsService()

    packing_request = PackSequence.Request()
    packing_request.objects_to_pick = 'Box_Gluehlampe,Box_Wischblatt,Keilriemen_gross,Box_Bremsbacke,Keilriemen_klein,Tuete'

    future = node.packing_client.call_async(packing_request)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


##
#ros2 service call items_to_pack aip_packing_planning_interfaces/srv/PackItems '{}'