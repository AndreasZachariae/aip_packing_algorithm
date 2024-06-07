import rclpy
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from aip_packing_planning_interfaces.srv import PackSequence
from Pack_Algorithm import Packplan
from pkg_pack_node.Pack_Algorithm import Pack_Algorithm
from pkg_pack_node.Classes.request_transfer import request_transfer


class PackingPlanningServer(Node):

    def __init__(self):
        super().__init__('packing_planning_server') # Initialisiere Node
        self.srv = self.create_service(PackSequence, 'packing_planning', self.packing_planning_callback) # Initialisiere Service


    def packing_planning_callback(self, request, response): 
        self.get_logger().info('Planning packing position...')
        
        # Setze Request
        self.get_logger().info('Request: \n' + str(request))
        request_transfer.set_request(request)

        # Starte Packplanung
        Pack_Algorithm()

        print("Generate Output done")
        packplan = pd.DataFrame(Packplan.get_packplan())
        self.get_logger().info('Packplan: \n' + str(packplan))

        # Erstelle Response



        return response
    
def main(args=None):
    rclpy.init(args=args)
    packing_planning_server = PackingPlanningServer() # Initialisiere Server
    rclpy.spin(packing_planning_server) # Starte Server
    packing_planning_server.destroy_node() # Beende Server
    rclpy.shutdown() # Beende Node

if __name__ == '__main__':
    main()
