import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from aip_packing_planning_interfaces.srv import PackItems
from Classes.items_transfer import items_transfer
from aip_packing_planning_interfaces.srv import PackSequence


class PackingPlanningClient(Node):

    def __init__(self):
        super().__init__('pack_planning_client')
        # zu PackSequence ändern
        self.cli = self.create_client(PackItems, 'pack_planning')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # zu PackSequence ändern
        self.req = PackItems.Request()
        print("Init finished")

    def send_request(self):

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    client = PackingPlanningClient()
    
    client.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(client)
        print("Requesting...")
        if client.future.done():
            try:
                response = client.future.result()
                print(response)
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Result: %r' % (response.objects_to_pick))
            break

    client.destroy_node()
    rclpy.shutdown()
    
    
    #### Processing the response ####
    items = []
    item_data = response.objects_to_pick
    items = item_data.split(",")

    # Setze Request
    items_transfer.set_items(items)

    # Starte Packplanung
    from Pack_Algorithm import Pack_Algorithm
    # Pack_Algorithm()
    

if __name__ == '__main__':
    main()
