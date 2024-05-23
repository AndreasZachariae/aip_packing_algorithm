from pick_package.iras_utils_transform import Affine
from geometry_msgs.msg import Pose as PoseMsg



def affine_to_pose_msg(pose: Affine):
    pose_msg = PoseMsg()
    pose_msg.position.x = pose.translation[0]
    pose_msg.position.y = pose.translation[1]
    pose_msg.position.z = pose.translation[2]
    pose_msg.orientation.x = pose.quat[0]
    pose_msg.orientation.y = pose.quat[1]
    pose_msg.orientation.z = pose.quat[2]
    pose_msg.orientation.w = pose.quat[3]
    return pose_msg

