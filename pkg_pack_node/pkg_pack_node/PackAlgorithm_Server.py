import sys
sys.path.append('/home/docker/ros2_ws/src/pkg_pack_node/pkg_pack_node')
import rclpy
from rclpy.node import Node
from aip_packing_planning_interfaces.srv import PackSequence
from aip_packing_planning_interfaces.msg import Package
from aip_packing_planning_interfaces.msg import SolutionFeedback
from geometry_msgs.msg import Vector3, Point
from Classes.items_transfer import items_transfer
from Classes.packplan import Packplan
import cv2
from cv_bridge import CvBridge
from Classes.container_amount_transfer import container_amount_transfer

class PackItemsService(Node):

    def __init__(self):
        super().__init__('pack_items_service')
        self.srv = self.create_service(PackSequence, 'pack_planning', self.pack_items_callback)
        
        # self.solution_feedback_publisher = self.create_publisher(SolutionFeedback, 'solution_feedback', 10)

        self.get_logger().info('Service server is ready.')


    def pack_items_callback(self, request, response):

        items = []
        
        # Aktiviere folgende Zeile für Testzwecke
        # items = ["Box_Gluehlampe", "Box_Bremsbacke", "Keilriemen_klein", "Tuete"]
        
        # Aktiviere folgende Zeile für normale Funktionalität
        items = request.objects_to_pick

        # print("Items: ", items)

        # Setze Request
        items_transfer.set_items(items)

        # Starte Packplanung
        from Pack_Algorithm import Pack_Algorithm
        # Pack_Algorithm()
    
        result = Packplan.get_packplan()
        
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

        print("Response:\n", response)
        
        
        # Solution Image Feeback erstellen

        # Lade das Bild, das die Lösung zeigt
        cv_image = cv2.imread("../solution_screenshot.png")

        # Überprüfen, ob das Bild erfolgreich geladen wurde
        if cv_image is None:
            raise IOError("Bild konnte nicht geladen werden")

        # Konvertiere das OpenCV-Bild in eine ROS2 Image-Nachricht
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        
        # test = bridge.imgmsg_to_cv2(image_msg, "passthrough")  # Konvertiere die ROS 2 Image-Nachricht zurück in ein OpenCV-Bild
        
        
        # Publisher für Übergabe des Containermengenstatus
        container_amount = container_amount_transfer.get_container_amount()
        print("\nContainermenge: ", container_amount)
        
        if container_amount > 1:
            container_amount_status = "Containermenge größer als 1, es werden nicht alle Pakete gepackt."
            print("Containermenge größer als 1, es werden nicht alle Pakete gepackt.")
        else:
            container_amount_status = "Alle Pakete werden gepackt."
            print("Alle Pakete werden gepackt.")


        # Erstelle SolutionFeedback-Nachricht
        # feedback = SolutionFeedback()
        response.feedback.image = image_msg
        response.feedback.message = container_amount_status

        # self.solution_feedback_publisher.publish(feedback)
        print("Image and Status added to response.")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PackItemsService()

    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
