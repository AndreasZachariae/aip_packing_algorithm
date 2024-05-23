import rclpy
from rclpy.node import Node
from iras_srvs.srv import Pose as PoseSrv
from iras_srvs.srv import AttachObject
from iras_srvs.srv import DetachObject
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

def wait_for_response(future, minimal_client):
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
                return None
            else:
                return response

class RobotClient(Node):
    def __init__(self, sim=True):
        super().__init__('minimal_client_async')
        self.sim = sim
        self.move_cli = self.create_client(PoseSrv, '/move_to_pose')
        self.attach_cli = self.create_client(AttachObject, '/attach_object')
        self.detach_cli = self.create_client(DetachObject, '/detach_object')

        while not self.move_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('move_to_pose service not available, waiting again...')
        while not self.attach_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('attach_object service not available, waiting again...')
        while not self.detach_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('detach_object service not available, waiting again...')

        self.req_move = PoseSrv.Request()
        self.req_attach = AttachObject.Request()
        self.req_detach =  DetachObject.Request()
        #if not self.sim:
        #TODO
            
            

    def send_move_request(self, pose, cart=False, testWindow = False):
        self.req_move.pose = pose
        self.req_move.cart = cart
        self.future = self.move_cli.call_async(self.req_move)
        rclpy.spin_until_future_complete(self, self.future)
        response = wait_for_response(self.future, self)
        if testWindow == True:
            cv2.imshow("test", np.random.random([255, 255, 3]))
            cv2.waitKey(0)
        return response

    def send_attach_request(self, attached_object, testWindow = False):
        self.req_attach.attached_object_details = attached_object
        self.future = self.attach_cli.call_async(self.req_attach)
        rclpy.spin_until_future_complete(self, self.future)
        response = wait_for_response(self.future, self)
        if testWindow == True:
            cv2.imshow("test", np.random.random([255, 255, 3]))
            cv2.waitKey(0)
        return response
    
    def send_detach_request(self, detach_object, testWindow = False):
        self.req_detach.detached_object_details = detach_object
        self.future = self.detach_cli.call_async(self.req_detach)
        rclpy.spin_until_future_complete(self, self.future)
        response = wait_for_response(self.future, self)
        if testWindow == True:
            cv2.imshow("test", np.random.random([255, 255, 3]))
            cv2.waitKey(0)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    robot_client = RobotClient()
    req_attach = AttachObject.Request()
    attached_object = req_attach.attached_object_details
    attached_object.link_name = "tool0"
    attached_object.object.header.frame_id = "tool0"
    attached_object.object.id = "box"
    pose = Pose()
    pose.position.z = 0.1
    pose.orientation.w = 1.0
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    primitive.dimensions = [0.0] * 3
    primitive.dimensions[0] = 0.075
    primitive.dimensions[1] = 0.075
    primitive.dimensions[2] = 0.075
    
    attached_object.object.primitives.insert(len(attached_object.object.primitives),primitive)
    attached_object.object.primitive_poses.insert(len(attached_object.object.primitive_poses),pose)
    attached_object.object.operation = attached_object.object.ADD
    attached_object.touch_links = ["tool0"]
    
    cv2.imshow("test", np.random.random([255, 255, 3]))
    cv2.waitKey(0)
    
    response_attach = robot_client.send_attach_request(attached_object)
    robot_client.get_logger().info(
                    'Result of service attach_object %d' % 
                    (response_attach.attach_success))
    cv2.imshow("test", np.random.random([255, 255, 3]))
    cv2.waitKey(0)
    
    # move object attached
    pose_to_move = Pose()
    pose_to_move.orientation.w = -0.056
    pose_to_move.orientation.x = -0.38
    pose_to_move.orientation.y = 0.65
    pose_to_move.orientation.z = -0.64
    pose_to_move.position.x = 1.5113
    pose_to_move.position.y = -2.2602
    pose_to_move.position.z = 2.1683
    
    response_move = robot_client.send_move_request(pose_to_move)
    robot_client.get_logger().info(
                    'Result of service move_to_pose %d' % 
                    ( response_move.success))
    cv2.imshow("test", np.random.random([255, 255, 3]))
    cv2.waitKey(0)
    
    # detach object details
    req_detach = DetachObject.Request()
    detach_object = req_detach.detached_object_details
    detach_object.object.id = "box"
    detach_object.link_name = "tool0"
    
    response_detach = robot_client.send_detach_request(detach_object)
    robot_client.get_logger().info(
                    'Result of service detach_object %d' % 
                    ( response_detach.detach_success))
    cv2.imshow("test", np.random.random([255, 255, 3]))
    cv2.waitKey(0)

    robot_client.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
