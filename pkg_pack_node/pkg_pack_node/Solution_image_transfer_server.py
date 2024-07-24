import sys
sys.path.append('/home/docker/ros2_ws/src/pkg_pack_node/pkg_pack_node')
import rclpy
from rclpy.node import Node
from aip_packing_planning_interfaces.srv import SolutionImage
import cv2
from cv_bridge import CvBridge


class SolutionImageTransferService(Node):

    def __init__(self):
        super().__init__('solution_image_transfer_service')
        self.srv = self.create_service(SolutionImage, 'solution_image_transfer', self.solution_image_transfer_callback)

        self.get_logger().info('Service server is ready.')
    
    
    def solution_image_transfer_callback(self, request, response):

        cv_image = cv2.imread("solution_screenshot.png")

        # Überprüfen, ob das Bild erfolgreich geladen wurde
        if cv_image is None:
            raise IOError("Bild konnte nicht geladen werden")

        # Konvertiere das OpenCV-Bild in eine ROS2 Image-Nachricht
        bridge = CvBridge()
        image_msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        test = bridge.imgmsg_to_cv2(image_msg, "passthrough")  # Konvertiere die ROS 2 Image-Nachricht zurück in ein OpenCV-Bild
        
        response.image = image_msg

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SolutionImageTransferService()

    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
