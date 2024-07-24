import sys
sys.path.append('/home/docker/ros2_ws/src/pkg_pack_node/pkg_pack_node')
import rclpy
from rclpy.node import Node
from aip_packing_planning_interfaces.srv import PackSequence
from aip_packing_planning_interfaces.msg import Package
from geometry_msgs.msg import Vector3, Point
from Classes.items_transfer import items_transfer
from Classes.packplan import Packplan


class PackItemsService(Node):

    def __init__(self):
        super().__init__('pack_items_service')
        self.srv = self.create_service(PackSequence, 'pack_planning', self.pack_items_callback)
        # self.packing_client = self.create_client(PackSequence, 'pack_planning')

        self.get_logger().info('Service server is ready.')
        print("Service Server is running...")



    def pack_items_callback(self, request, response):

        items = []
        
        # Aktiviere folgende Zeile für Testzwecke
        items = ["Box_Gluehlampe", "Box_Wischblatt", "Keilriemen_gross", "Box_Bremsbacke", "Keilriemen_klein", "Tuete"]
        
        # Aktiviere folgende Zeile für normale Funktionalität
        # items = request.objects_to_pick

        print("Items: ", items)

        # Setze Request
        items_transfer.set_items(items)

        # Starte Packplanung
        from Pack_Algorithm import Pack_Algorithm
        # Pack_Algorithm()
    
        result = Packplan.get_packplan()
        
        # response = PackSequence.Response()
        
        # Separiere den Packplan

        class_name = result['label_odtf'].tolist()
        dimensions = result[['length', 'width', 'height']].values.tolist()
        weight = result['weight'].tolist()
        rotation_index = result['rotation_index'].tolist()
        place_coordinates = result[['x_pack', 'y_pack', 'z_pack']].values.tolist()
        for i in range(len(class_name)):
            package = Package()
            package.class_name = class_name[i]
            package.dimensions = Vector3()
            package.dimensions.x = dimensions[i][0]
            package.dimensions.y = dimensions[i][1]
            package.dimensions.z = dimensions[i][2]
            package.weight = weight[i]
            package.rotation_index = rotation_index[i]
            package.place_coordinates = Point()
            package.place_coordinates.x = place_coordinates[i][0]
            package.place_coordinates.y = place_coordinates[i][1]
            package.place_coordinates.z = place_coordinates[i][2]
            response.package.packages.append(package)

        # Erstellen des Response

        print("Response:\n", response)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PackItemsService()

    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
