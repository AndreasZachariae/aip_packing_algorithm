import rclpy
from geometry_msgs.msg import Pose as PoseMsg
import copy
from pick_package.robot_client import RobotClient
from pick_package.iras_utils_transform import Affine
from pick_package.util import affine_to_pose_msg
import numpy as np
import time
from std_msgs.msg import Float64MultiArray
import pandas as pd
import os
from iras_srvs.srv import AttachObject, DetachObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


# table:
# x = 0.4 + 0.017 + 0.565 (incl 0.045) + 0.318 = 1.3 / Ablage 0.332 - 0.09 + 0.105 (X-Position)
# y = bei Roboter: 0.201 + 0.4 + 0.205 = 0.806 / bei Container: 0.146 + 0.045 + 0.615 = 0.806 / Ablage 0.105
# z = 0.94 + 0.025 = 0,965 

# cylinder extended
gripper_extension = 0.15 

# order name
order = '2023-02-14 14h33m04s'


# corner of the shelf where the first package is located
s_corner_x = 0.347 
s_corner_y = 0.911 
s_corner_z = 0.94 

# corner of the container (outside)
c_corner_x = 0.937 
c_corner_y = 0.191 
c_corner_z = 0.94 





def read_csv_packplan():
    # read packplan from csv
    # first 3 columns: dimension parralel to container length, heigth, width
    column_names = ['parallel_c_length', 'height', 'parallel_c_width', 'x', 'z', 'y', 'is_stack', 'package_ID', 'package_sequence_nr', 'weight']
    packplan = pd.read_csv('src/pick_package/Packalgorithmus/Simulationen/' + order + ' BI01 Simulation Packplan.csv', names = column_names)
    
    # convert mm into m
    packplan.parallel_c_length = packplan.parallel_c_length / 1000
    packplan.height = packplan.height / 1000
    packplan.parallel_c_width = packplan.parallel_c_width / 1000
    packplan.x = packplan.x / 1000
    packplan.y = packplan.y / 1000
    packplan.z = packplan.z / 1000
    packplan.weight = packplan.weight / 1000

    # define the package length, width and rotation on the basis of the orientations
    packplan = pd.DataFrame(packplan)
    length = []
    width = []
    rotation_index = []
    for i in range(len(packplan)):
        if packplan.parallel_c_width[i] < packplan.parallel_c_length[i]: 
            # orientation 1
            length.append(packplan.parallel_c_length[i])
            width.append(packplan.parallel_c_width[i])
            rotation_index.append(0)
        else:
            # orientation 3
            length.append(packplan.parallel_c_width[i])
            width.append(packplan.parallel_c_length[i])
            rotation_index.append(1)

    # insert the length and width as columns into the packplan list
    packplan.insert(10, 'length', length)
    packplan.insert(11, 'width', width)

    return packplan, rotation_index



def read_csv_container():
    # read which container is used
    column_names = ['width', 'height', 'length']
    container = pd.read_csv('src/pick_package/Packalgorithmus/Simulationen/' + order + ' BI01 Simulation Container.csv', names = column_names)
    # convert mm into m
    container.width = container.width / 1000
    container.height = container.height / 1000
    container.length = container.length / 1000

    
    if container.width[0] == 0.232:
        #container X01
        container_outside_x = 0.166
        container_inside_x = container.length[0]
        container_outside_y = 0.246 
        container_inside_y = container.width[0]
        container_outside_z = 0.123
        container_inside_z = container.height[0]
    elif container.width[0] == 0.252:
        #container X02
        container_outside_x = 0.216
        container_inside_x = container.length[0]
        container_outside_y = 0.266 
        container_inside_y = container.width[0]
        container_outside_z = 0.183
        container_inside_z = container.height[0]
    elif container.width[0] == 0.304:
        #container X03
        container_outside_x = 0.266
        container_inside_x = container.length[0]
        container_outside_y = 0.316 
        container_inside_y = container.width[0]
        container_outside_z = 0.243
        container_inside_z = container.height[0]
    elif container.width[0] == 0.400:
        #container X04
        container_outside_x = 0.312
        container_inside_x = container.length[0]
        container_outside_y = 0.412
        container_inside_y = container.width[0]
        container_outside_z = 0.225
        container_inside_z = container.height[0]
    elif container.width[0] == 0.435:
        #container X05
        container_outside_x = 0.407
        container_inside_x = container.length[0]
        container_outside_y = 0.452 
        container_inside_y = container.width[0]
        container_outside_z = 0.321
        container_inside_z = container.height[0]
    elif container.width[0] == 0.716:
        #container X06
        container_outside_x = 0.556
        container_inside_x = container.length[0]
        container_outside_y = 0.736
        container_inside_y = container.width[0]
        container_outside_z = 0.303
        container_inside_z = container.height[0]
    elif container.width[0] == 0.540:
        #container X07
        container_outside_x = 0.385
        container_inside_x = container.length[0]
        container_outside_y = 0.566
        container_inside_y = container.width[0]
        container_outside_z = 0.300
        container_inside_z = container.height[0]
    elif container.width[0] == 0.760:
        #container X08
        container_outside_x = 0.380
        container_inside_x = container.length[0]
        container_outside_y = 0.780
        container_inside_y = container.width[0]
        container_outside_z = 0.300
        container_inside_z = container.height[0]
    elif container.width[0] == 0.732:
        #container X22
        container_outside_x = 0.261
        container_inside_x = container.length[0]
        container_outside_y = 0.746
        container_inside_y = container.width[0]
        container_outside_z = 0.087
        container_inside_z = container.height[0]
    else: print("No container with these dimensions")

    # wall thickness of the container
    c_wall_x = (container_outside_x - container_inside_x) / 2
    c_wall_y = (container_outside_y - container_inside_y) / 2
    c_wall_z = (container_outside_z - container_inside_z)

    return c_wall_x, c_wall_y, c_wall_z, container


def choose_cylinder(packplan):
    # cylinder messages:
    # msg_data type: [cylinder_back_left, cylinder_back_right, cylinder_front_left, cylinder_front_right]
    global msg_cylinder
    global index_msgs
    global tcps_cylinder
    msg0 = [0.0, 0.0, 0.0, 0.0]
    msg1 = [gripper_extension, 0.0, 0.0, 0.0]
    msg2 = [0.0, gripper_extension, 0.0, 0.0]
    msg3 = [0.0, 0.0, gripper_extension, 0.0]
    msg4 = [0.0, 0.0, 0.0, gripper_extension]
    msg5 = [gripper_extension, gripper_extension, 0.0, 0.0]
    msg6 = [0.0, gripper_extension, 0.0, gripper_extension]
    msg7 = [gripper_extension, gripper_extension, 0.0, gripper_extension]
    msg_cylinder = [msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7]

    # choice of gripper cylinders (assumption: package_length > package_width)
    # index_msg = index for msg_cylinder
    # tcp_cylinder = definition of the new tcp which is at the bottom of the extended cylinder (in the middle of the combination)
    index_msgs = []
    tcps_cylinder = []

    for i in range(len(packplan)):
        if packplan.width[i] < 0.051 and packplan.length[i] < 0.051 and packplan.weight[i] < 0.09:
            index_msg = 3
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [-0.075, 0.106, 0.281], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.051 and packplan.width[i] < 0.092 and packplan.length[i] > 0.051 and packplan.length[i] < 0.129 and packplan.weight[i] < 1.14:
            index_msg = 1
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [0.0485, 0.0815, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.051 and packplan.width[i] < 0.100 and packplan.length[i] > 0.129 and packplan.length[i] < 0.232 and packplan.weight[i] < 2.27:
            index_msg = 2
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.035 and packplan.width[i] < 0.151 and packplan.length[i] > 0.165 and packplan.length[i] < 0.232 and packplan.weight[i] < 3.10:
            index_msg = 4 
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [-0.0585, -0.055, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.053 and packplan.width[i] < 0.182 and packplan.length[i] > 0.232 and packplan.weight[i] < 3.41:
            index_msg = 5
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [0.049, -0.009, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.151:
            if packplan.length[i] < 0.165 and packplan.weight[i] < 2.27:
                index_msg = 2
                index_msgs.append(index_msg)
                tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)
            elif packplan.length[i] > 0.165 and packplan.length[i] < 0.243 and packplan.weight[i] < 5.37:
                index_msg = 6
                index_msgs.append(index_msg)
                tcp_cylinder =  Affine(translation = [-0.001, -0.054, 0.299], rotation=[0.0, 0.0, 0.0]) 
                tcps_cylinder.append(tcp_cylinder)
            elif packplan.length[i] > 0.243 and packplan.weight[i] < 6.51:
                index_msg = 7
                index_msgs.append(index_msg)
                tcp_cylinder =  Affine(translation = [-0.001, -0.015, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)
            else: print("Error: package dimensions not allowable")
        else: print("Error: package dimensions not allowable")

    return index_msgs, tcps_cylinder
    




def collision_cor(packplan, i):
    # biggest gripper dimensions (collision)
    gripper_x = 0.0945 * 2
    gripper_y = 0.14 * 2
    gripper_z = 0.299
    # corrections
    # correction that the package does not collide with the container: gap of 1mm
    #col_correction = 0.001
    # gripper overhang --> correction
    if (gripper_x / 2) > (packplan.width[i] / 2):
        g_correction_x = (gripper_x - packplan.width[i]) / 2
    else:
        g_correction_x = 0.0

    if (gripper_y / 2) > (packplan.length[i] / 2):
        g_correction_y = (gripper_y - packplan.length[i]) / 2
    else:
        g_correction_y = 0.0

    corr_width = g_correction_x
    corr_length = g_correction_y
    

    return corr_width, corr_length

def collision_correction(packplan, index_msgs, rotation_index):
    # correction of the gripper overhang
    corr_width = []
    corr_length = []
    for i in range(len(packplan)):
        if index_msgs[i] == 1:
            # cylinder back left
            if packplan.width[i] < 0.086:
                correction_x = (0.086 - packplan.width[i]) / 2
            else: correction_x = 0.0
            if packplan.length[i] < 0.132:
                correction_y = (0.132 - packplan.length[i]) / 2
            else: correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
        elif index_msgs[i] == 2:
            # cylinder back right
            if packplan.width[i] < 0.092:
                correction_x = (0.092 - packplan.width[i]) / 2
            else: correction_x = 0.0
            correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
        elif index_msgs[i] == 3:
            # cylinder front left
            if packplan.width[i] < 0.132:
                correction_x = (0.132 - packplan.width[i]) / 2
            else: correction_x = 0.0
            if packplan.length[i] < 0.166:
                correction_y = (0.166 - packplan.length[i]) / 2
            else: correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
        elif index_msgs[i] == 4:
            # cylinder front right
            if packplan.width[i] < 0.1:
                correction_x = (0.1 - packplan.width[i]) / 2
            else: correction_x = 0.0
            correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
        elif index_msgs[i] == 5:
            # cylinder back right, cylinder back left
            if packplan.width[i] < 0.106:
                correction_x = (0.106 - packplan.width[i]) / 2
            else: correction_x = 0.0
            correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
        elif index_msgs[i] == 6:
            # cylinder back right, cylinder front right
            if packplan.width[i] < 0.182:
                correction_x = (0.182 - packplan.width[i]) / 2
            else: correction_x = 0.0
            correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
        elif index_msgs[i] == 7:
            # cylinder back left, cylinder back right, cylinder front right
            if packplan.width[i] < 0.182:
                correction_x = (0.182 - packplan.width[i]) / 2
            else: correction_x = 0.0
            correction_y = 0.0
            corr_width.append(correction_x)
            corr_length.append(correction_y)
    
    # calculation of the total correction
    total_corr_x = []
    total_corr_y = []

    if rotation_index[0] == 0:
        total_x = corr_width[0]
        total_y = corr_length[0]
    else:
        total_x = corr_length[0]
        total_y = corr_width[0]
    total_corr_x.append(total_x)
    total_corr_y.append(total_y)

    for i in range(1, len(packplan)):
        if rotation_index[i] == 0:
            if packplan.y[i] == 0:
                total_x = corr_width[i]
            else: 
                total_x = total_corr_x[i-1] + corr_width[i]
            if packplan.x[i] == 0:
                total_y = corr_length[i]
            else:
                total_y = total_corr_y[i-1] + corr_length[i]
            total_corr_x.append(total_x)
            total_corr_y.append(total_y)
        else:
            if packplan.y[i] == 0:
                total_x = corr_length[i]
            else:
                total_x = total_corr_y[i-1] + corr_length[i]
            if packplan.x[i] == 0:
                total_y = corr_width[i]
            else:
                total_y = total_corr_x[i-1] + corr_width[i]
            total_corr_x.append(total_x)
            total_corr_y.append(total_y)
    
    
    # if rotation_index[0] == 0:
    #     total_corr_x.append(corr_width[0])
    #     total_corr_y.append(corr_length[0])
    # else: 
    #     total_corr_x.append(corr_length[0])
    #     total_corr_y.append(corr_width[0])
    
    # for i in range(1, len(packplan)):
    #     if rotation_index[i] == 0:
    #         total_x = total_corr_x[i-1] + corr_width[i]
    #         total_y = total_corr_y[i-1] + corr_length[i]
    #         total_corr_x.append(total_x)
    #         total_corr_y.append(total_y)
    #     else:
    #         total_x = total_corr_y[i-1] + corr_length[i]
    #         total_y = total_corr_x[i-1] + corr_width[i]
    #         total_corr_x.append(total_x)
    #         total_corr_y.append(total_y)


    return total_corr_x, total_corr_y




def calculate_poses(packplan, total_corr_x, total_corr_y, rotation_index, c_wall_x, c_wall_y, c_wall_z, container):
    package_pick_poses = []
    package_place_poses = []

    # pick poses
    pick_ref_pose = Affine(translation = [s_corner_x + (packplan.width[0] / 2), s_corner_y + (packplan.length[0] / 2), s_corner_z + packplan.height[0] + gripper_extension],
                        rotation=[0.0, 0.0, 0.0])
    
    pose = copy.deepcopy(pick_ref_pose)
    package_pick_poses.append(pose)
    for i in range(1, len(packplan)):
        package_pick_offset = Affine(translation = [(packplan.width[i-1] / 2) + (packplan.width[i] / 2), (packplan.length[i] / 2) - (packplan.length[i-1] / 2), packplan.height[i] - packplan.height[i-1]], rotation = [0, 0, 0])
        pose = package_pick_offset * pose
        package_pick_poses.append(pose)
    
    # place poses    
    for i in range(len(packplan)):
        if rotation_index[i] == 0:
            place_pose = Affine(translation = [c_corner_x - c_wall_x - container.length[0] + total_corr_x[i] + packplan.y[i] + (packplan.parallel_c_width[i] / 2), c_corner_y + c_wall_y + packplan.x[i] + (packplan.parallel_c_length[i] / 2) + total_corr_y[i], c_corner_z + c_wall_z + packplan.z[i] + (packplan.height[i] / 2) + gripper_extension], 
                    rotation=[0.0, 0.0, 0.0])
        else:
            place_pose = Affine(translation = [c_corner_x - c_wall_x - container.length[0] + total_corr_x[i] + packplan.y[i] + (packplan.parallel_c_width[i] / 2), c_corner_y + c_wall_y + packplan.x[i] + (packplan.parallel_c_length[i] / 2) + total_corr_y[i], c_corner_z + c_wall_z + packplan.z[i] + (packplan.height[i] / 2) + gripper_extension], 
                    rotation=[0.0, 0.0, 90/180 * np.pi])
        package_place_poses.append(place_pose)


    return package_pick_poses, package_place_poses




def calculate_attach_poses(packplan, package_pick_poses, package_place_poses):
    # calculates poses where the packages are attached, like the Pick poses (except z-direction), but separated into x, y, z instead of Affine transformation
    # positions where packages are attached to the gripper
    attach_position_x = []
    attach_position_y = []
    attach_position_z = []
    # positions where packages are detached and attached to the container
    detach_position_x = []
    detach_position_y = []
    detach_position_z = []

    
    for i in range(len(packplan)):
        # calculate poses where packages are attached to the gripper
        pick_pose = affine_to_pose_msg(package_pick_poses[i])
        pick_x = pick_pose.position.x
        pick_y = pick_pose.position.y
        pick_z = pick_pose.position.z - gripper_extension - (packplan.height[i] / 2)
        attach_position_x.append(pick_x)
        attach_position_y.append(pick_y)
        attach_position_z.append(pick_z)

        # calculate poses where packages are attached to the container
        place_pose = affine_to_pose_msg(package_place_poses[i])
        place_x = place_pose.position.x
        place_y = place_pose.position.y
        place_z = place_pose.position.z - gripper_extension 
        detach_position_x.append(place_x)
        detach_position_y.append(place_y)
        detach_position_z.append(place_z)

    print("attach z")
    print(attach_position_z[0])
    print("detach z")
    print(detach_position_z[0])

    

    return attach_position_x, attach_position_y, attach_position_z, detach_position_x, detach_position_y, detach_position_z







def load_container_cylinder(args=None):
    sim = True
    rclpy.init(args = args)
    time.sleep(1)
    minimal_client = RobotClient(sim)
    # create topic for cylinder movements
    publish_topic = "/gripper_controller/commands"
    minimal_client.publisher_ = minimal_client.create_publisher(Float64MultiArray, publish_topic, 1)
    msg = Float64MultiArray()
    

    # read packplan
    packplan, rotation_index = read_csv_packplan()
    print("Packplan")
    print(packplan)
    
    # read container from csv and calculate wall thickness of the container
    c_wall_x, c_wall_y, c_wall_z, container = read_csv_container()
    
    # choose cylinder and tcp
    index_msgs, tcps_cylinder = choose_cylinder(packplan)
    
    # calculate corrections due to collisions
    # corr_width, corr_length = collision_cor(packplan, i = 0)
    total_corr_x, total_corr_y = collision_correction(packplan, index_msgs, rotation_index)

    # calculate pick- and place-poses
    package_pick_poses, package_place_poses = calculate_poses(packplan, total_corr_x, total_corr_y, rotation_index, c_wall_x, c_wall_y, c_wall_z, container)
    
    # calculate attach-poses for attaching the package to the gripper and to the container
    attach_position_x, attach_position_y, attach_position_z, detach_position_x, detach_position_y, detach_position_z = calculate_attach_poses(packplan, package_pick_poses, package_place_poses)

    print(package_pick_poses)
    print("&")
    print(package_place_poses)
    

    for i in range(len(packplan)):
        
        if ((packplan.parallel_c_width[i] + packplan.y[i] + total_corr_x[i]) < container.length[0]) and ((packplan.parallel_c_length[i] + packplan.x[i] + total_corr_y[i]) < container.width[0]):

            # offset over pick- and place-poses for pre- and post-pick-poses
            pre_post_pick_offset = Affine(translation = [0, 0, container.height[0] + packplan.height[i]], rotation = [0, 0, 0])
            pre_post_place_offset = Affine(translation = [0, 0, container.height[0] + packplan.height[i]], rotation = [0, 0, 0])
            print("c wall z")
            print(c_wall_z)
            
            # Pick
            pre_pick_pose = pre_post_pick_offset * package_pick_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(pre_pick_pose), False, True) # move to pre_pick_pose
            pick_pose = package_pick_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(pick_pose), True, True) # move to pick_pose
            msg.data = msg_cylinder[index_msgs[i]]
            minimal_client.publisher_.publish(msg) # extend cylinder
            response = minimal_client.send_move_request(affine_to_pose_msg(pick_pose), True, True) # stay at pick_pose while cylinder extends
            # attach package to gripper
            req = AttachObject.Request()
            attached_object = req.attached_object_details
            index_string = str(index_msgs[i])
            i_string = str(i)
            attached_object.link_name = "tcp_" + index_string
            attached_object.object.header.frame_id = "base"
            attached_object.object.id = "box_" + i_string
            pose = Pose()
            pose.position.x = attach_position_x[i] 
            pose.position.y = attach_position_y[i] 
            pose.position.z = attach_position_z[i] + 0.0005
            primitive = SolidPrimitive()
            primitive.type = primitive.BOX
            primitive.dimensions = [0.0] * 3
            primitive.dimensions[0] = packplan.width[i] - 0.001
            primitive.dimensions[1] = packplan.length[i] - 0.001
            primitive.dimensions[2] = packplan.height[i] - 0.001
            attached_object.object.primitives.insert(len(attached_object.object.primitives),primitive)
            attached_object.object.primitive_poses.insert(len(attached_object.object.primitive_poses),pose)
            attached_object.object.operation = attached_object.object.ADD
            attached_object.touch_links = ["tcp_" + index_string]
            minimal_client.send_attach_request(attached_object, True)
            # end attach
            post_pick_pose = pre_post_pick_offset * package_pick_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(post_pick_pose), True, True) # move to post_pick_pose
            
            # Place
            pre_place_pose = pre_post_place_offset * package_place_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(pre_place_pose), False, True) # move to pre_place_pose
            place_pose = package_place_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(place_pose), True, True) # move to place_pose
            # detach package from gripper
            req_detach = DetachObject.Request()
            detach_object = req_detach.detached_object_details
            detach_object.object.id = "box_" + i_string
            detach_object.link_name = "tcp_" + index_string
            response_detach = minimal_client.send_detach_request(detach_object, True)
            # end detach

            # attach each package to the container that was already in the container
            req = AttachObject.Request()
            attached_object = req.attached_object_details
            attached_object.link_name = "container" 
            attached_object.object.header.frame_id = "base"
            attached_object.object.id = "box_" + i_string
            pose = Pose()
            pose.position.x = detach_position_x[i] 
            pose.position.y = detach_position_y[i] 
            pose.position.z = detach_position_z[i] 
            if rotation_index[i] == 0:
                pose.orientation.z = 0.0
            else:
                pose.orientation.z = 1.0
            primitive = SolidPrimitive()
            primitive.type = primitive.BOX
            primitive.dimensions = [0.0] * 3
            primitive.dimensions[0] = packplan.width[i] - 0.001
            primitive.dimensions[1] = packplan.length[i] - 0.001
            primitive.dimensions[2] = packplan.height[i] - 0.001
            attached_object.object.primitives.insert(len(attached_object.object.primitives),primitive)
            attached_object.object.primitive_poses.insert(len(attached_object.object.primitive_poses),pose)
            attached_object.object.operation = attached_object.object.ADD
            attached_object.touch_links = ["container"]
            minimal_client.send_attach_request(attached_object, True)
            # end attach
            msg.data = msg_cylinder[0]
            minimal_client.publisher_.publish(msg) # retract cylinder
            response = minimal_client.send_move_request(affine_to_pose_msg(place_pose), True, True) # stay at place_pose while cylinder retracts
        
            post_place_pose = pre_post_place_offset * package_place_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(post_place_pose), True, True) # move to post_place_pose
        
    
 
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    load_container_cylinder()





#______________________________old_version___________________________________________________________________________

# import rclpy
# from geometry_msgs.msg import Pose as PoseMsg
# import copy
# from pick_package.robot_client import RobotClient
# from pick_package.iras_utils_transform import Affine
# from pick_package.util import affine_to_pose_msg
# import numpy as np
# import time
# from std_msgs.msg import Float64MultiArray
# import pandas as pd
# import os
# from iras_srvs.srv import AttachObject, DetachObject
# from geometry_msgs.msg import Pose
# from shape_msgs.msg import SolidPrimitive


# # table:
# # x = 0.4 + 0.017 + 0.565 (incl 0.045) + 0.318 = 1.3 / Ablage 0.332 - 0.09 + 0.105 (X-Position)
# # y = 0.201 + 0.4 + 0.205 = 0.806 / 0.146 + 0.045 + 0.615 / Ablage 0.105
# # z = 0.94 + 0.025 = 0,965 

# # biggest gripper dimensions (collision)
# gripper_x = 0.0945 * 2
# gripper_y = 0.14 * 2
# gripper_z = 0.299
# gripper_extension = 0.15 # cylinder extended

# # order name
# order = '2023-02-14 14h33m04s'


# # corner of the shelf where the first package is located
# s_corner_x = 0.347 
# s_corner_y = 0.911 
# s_corner_z = 0.94 

# # corner of the container (outside)
# c_corner_x = 0.937 
# c_corner_y = 0.191 
# c_corner_z = 0.94 





# def read_csv_packplan():
#     # read packplan from csv
#     # first 3 columns: dimension parralel to container length, heigth, width
#     column_names = ['parallel_c_length', 'height', 'parallel_c_width', 'x', 'z', 'y', 'is_stack', 'package_ID', 'package_sequence_nr', 'weight']
#     packplan = pd.read_csv('src/pick_package/Packalgorithmus/Simulationen/2023-02-02 14h35m15s BI02 Simulation Packplan.csv' , names = column_names)
    
#     # convert mm into m
#     packplan.parallel_c_length = packplan.parallel_c_length / 1000
#     packplan.height = packplan.height / 1000
#     packplan.parallel_c_width = packplan.parallel_c_width / 1000
#     packplan.x = packplan.x / 1000
#     packplan.y = packplan.y / 1000
#     packplan.z = packplan.z / 1000
#     packplan.weight = packplan.weight / 1000

#     # define the package length, width and rotation on the basis of the orientations
#     packplan = pd.DataFrame(packplan)
#     length = []
#     width = []
#     rotation_index = []
#     for i in range(len(packplan)):
#         if packplan.parallel_c_width[i] < packplan.parallel_c_length[i]: 
#             # orientation 1
#             length.append(packplan.parallel_c_length[i])
#             width.append(packplan.parallel_c_width[i])
#             rotation_index.append(0)
#         else:
#             # orientation 3
#             length.append(packplan.parallel_c_width[i])
#             width.append(packplan.parallel_c_length[i])
#             rotation_index.append(1)

#     # insert the length and width as columns into the packplan list
#     packplan.insert(10, 'length', length)
#     packplan.insert(11, 'width', width)

#     return packplan, rotation_index

# def read_csv_container():
#     # read which container is used
#     column_names = ['width', 'height', 'length']
#     container = pd.read_csv('src/pick_package/Packalgorithmus/Simulationen/' + order + ' BI01 Simulation Container.csv', names = column_names)
#     # convert mm into m
#     container.width = container.width / 1000
#     container.height = container.height / 1000
#     container.length = container.length / 1000

    
#     if container.width[0] == 0.232:
#         #container X01
#         container_outside_x = 0.166
#         container_inside_x = container.length[0]
#         container_outside_y = 0.246 
#         container_inside_y = container.width[0]
#         container_outside_z = 0.123
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.252:
#         #container X02
#         container_outside_x = 0.216
#         container_inside_x = container.length[0]
#         container_outside_y = 0.266 
#         container_inside_y = container.width[0]
#         container_outside_z = 0.183
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.304:
#         #container X03
#         container_outside_x = 0.266
#         container_inside_x = container.length[0]
#         container_outside_y = 0.316 
#         container_inside_y = container.width[0]
#         container_outside_z = 0.243
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.400:
#         #container X04
#         container_outside_x = 0.312
#         container_inside_x = container.length[0]
#         container_outside_y = 0.412
#         container_inside_y = container.width[0]
#         container_outside_z = 0.225
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.435:
#         #container X05
#         container_outside_x = 0.407
#         container_inside_x = container.length[0]
#         container_outside_y = 0.452 
#         container_inside_y = container.width[0]
#         container_outside_z = 0.321
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.716:
#         #container X06
#         container_outside_x = 0.556
#         container_inside_x = container.length[0]
#         container_outside_y = 0.736
#         container_inside_y = container.width[0]
#         container_outside_z = 0.303
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.540:
#         #container X07
#         container_outside_x = 0.385
#         container_inside_x = container.length[0]
#         container_outside_y = 0.566
#         container_inside_y = container.width[0]
#         container_outside_z = 0.300
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.760:
#         #container X08
#         container_outside_x = 0.380
#         container_inside_x = container.length[0]
#         container_outside_y = 0.780
#         container_inside_y = container.width[0]
#         container_outside_z = 0.300
#         container_inside_z = container.height[0]
#     elif container.width[0] == 0.732:
#         #container X22
#         container_outside_x = 0.261
#         container_inside_x = container.length[0]
#         container_outside_y = 0.746
#         container_inside_y = container.width[0]
#         container_outside_z = 0.087
#         container_inside_z = container.height[0]
#     else: print("No container with these dimensions")

#     # wall thickness of the container
#     c_wall_x = (container_outside_x - container_inside_x) / 2
#     c_wall_y = (container_outside_y - container_inside_y) / 2
#     c_wall_z = (container_outside_z - container_inside_z)

#     return c_wall_x, c_wall_y, c_wall_z, container



# def collision_correction(packplan, i):
#     # corrections
#     # correction that the package does not collide with the container: gap of 1mm
#     col_correction = 0.001
#     # gripper overhang --> correction
#     if (gripper_x / 2) > (packplan.width[i] / 2):
#         g_correction_x = (gripper_x - packplan.width[i]) / 2
#     else:
#         g_correction_x = 0.0

#     if (gripper_y / 2) > (packplan.length[i] / 2):
#         g_correction_y = (gripper_y - packplan.length[i]) / 2
#     else:
#         g_correction_y = 0.0

#     corr_width = col_correction + g_correction_x
#     corr_length = col_correction + g_correction_y
#     corr_height = col_correction

#     return col_correction, corr_width, corr_length, corr_height




# def calculate_poses(packplan, col_correction, corr_width, corr_length, corr_height, rotation_index, c_wall_x, c_wall_y, c_wall_z, container):
#     package_pick_poses = []
#     package_place_poses = []

#     # pick poses
#     pick_ref_pose = Affine(translation = [s_corner_x + (packplan.width[0] / 2) + col_correction, s_corner_y + (packplan.length[0] / 2) + col_correction, s_corner_z + col_correction + packplan.height[0] + gripper_extension],
#                         rotation=[0.0, 0.0, 0.0])
    
#     pose = copy.deepcopy(pick_ref_pose)
#     package_pick_poses.append(pose)
#     for i in range(1, len(packplan)):
#         package_pick_offset = Affine(translation = [(packplan.width[i-1] / 2) + (packplan.width[i] / 2), (packplan.length[i] / 2) - (packplan.length[i-1] / 2), packplan.height[i] - packplan.height[i-1]], rotation = [0, 0, 0])
#         pose = package_pick_offset * pose
#         package_pick_poses.append(pose)
    
#     # place poses
#     # rotation
#     rotation = [Affine(translation = [0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0]), Affine(translation = [0.0, 0.0, 0.0], rotation=[0.0, 0.0, 90/180 * np.pi])]
#     for i in range(len(packplan)):
#         place_position = Affine(translation = [c_corner_x - c_wall_x - container.length[0] + corr_length + packplan.y[i] + (packplan.parallel_c_width[i] / 2), c_corner_y + c_wall_y + packplan.x[i] + (packplan.parallel_c_length[i] / 2) + corr_width, c_corner_z + c_wall_z + corr_height + packplan.z[i] + packplan.height[i] + gripper_extension], 
#                     rotation=[0.0, 0.0, 0.0])
#         place_pose = place_position * rotation[rotation_index[i]]
#         package_place_poses.append(place_pose)


#     return package_pick_poses, package_place_poses




# def choose_cylinder(packplan, i):
#     # cylinder messages:
#     # msg_data type: [cylinder_back_left, cylinder_back_right, cylinder_front_left, cylinder_front_right]
#     global msg_cylinder
#     global index_msg
#     global tcp_cylinder
#     msg0 = [0.0, 0.0, 0.0, 0.0]
#     msg1 = [gripper_extension, 0.0, 0.0, 0.0]
#     msg2 = [0.0, gripper_extension, 0.0, 0.0]
#     msg3 = [0.0, 0.0, gripper_extension, 0.0]
#     msg4 = [0.0, 0.0, 0.0, gripper_extension]
#     msg12 = [gripper_extension, gripper_extension, 0.0, 0.0]
#     msg24 = [0.0, gripper_extension, 0.0, gripper_extension]
#     msg124 = [gripper_extension, gripper_extension, 0.0, gripper_extension]
#     msg_cylinder = [msg0, msg1, msg2, msg3, msg4, msg12, msg24, msg124]

#     # choice of gripper cylinders (assumption: package_length > package_width)
#     # index_msg = index for msg_cylinder
#     # tcp_cylinder = definition of the new tcp which is at the bottom of the extended cylinder (in the middle of the combination)
#     index_msg = 0
#     tcp_cylinder = Affine(translation = [0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0])


#     if packplan.width[i] < 0.068 and packplan.length[i] < 0.051 and packplan.weight[i] < 0.09:
#         index_msg = 3
#         tcp_cylinder =  Affine(translation = [-0.075, 0.106, 0.281], rotation=[0.0, 0.0, 0.0])
#     elif packplan.width[i] > 0.051 and packplan.width[i] < 0.068 and packplan.length[i] > 0.051 and packplan.length[i] < 0.129 and packplan.weight[i] < 1.14:
#         index_msg = 1
#         tcp_cylinder =  Affine(translation = [0.0485, 0.0815, 0.299], rotation=[0.0, 0.0, 0.0])
#     elif packplan.width[i] > 0.051 and packplan.width[i] < 0.068 and packplan.length[i] > 0.129 and packplan.length[i] < 0.232 and packplan.weight[i] < 2.27:
#         index_msg = 2
#         tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
#     elif packplan.width[i] > 0.035 and packplan.width[i] < 0.151 and packplan.length[i] > 0.165 and packplan.length[i] < 0.232 and packplan.weight[i] < 3.10:
#         index_msg = 4 
#         tcp_cylinder =  Affine(translation = [-0.0585, -0.055, 0.299], rotation=[0.0, 0.0, 0.0])
#     elif packplan.width[i] > 0.053 and packplan.width[i] < 0.151 and packplan.length[i] > 0.232 and packplan.weight[i] < 3.41:
#         index_msg = 5
#         tcp_cylinder =  Affine(translation = [0.049, -0.009, 0.299], rotation=[0.0, 0.0, 0.0])
#     elif packplan.width[i] > 0.151:
#         if packplan.length[i] < 0.165 and packplan.weight[i] < 2.27:
#             index_msg = 2
#             tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
#         elif packplan.length[i] > 0.165 and packplan.length[i] < 0.243 and packplan.weight[i] < 5.37:
#             index_msg = 6
#             tcp_cylinder =  Affine(translation = [-0.001, -0.054, 0.299], rotation=[0.0, 0.0, 0.0]) 
#         elif packplan.length[i] > 0.243 and packplan.weight[i] < 6.51:
#             index_msg = 7
#             tcp_cylinder =  Affine(translation = [-0.001, -0.015, 0.299], rotation=[0.0, 0.0, 0.0])
#         else: print("Error: package dimensions not allowable")
#     else: print("Error: package dimensions not allowable")

#     return index_msg, tcp_cylinder
    



# def load_container_cylinder(args=None):
#     sim = True
#     rclpy.init(args = args)
#     time.sleep(1)
#     minimal_client = RobotClient(sim)
#     # create topic for cylinder movements
#     publish_topic = "/gripper_controller/commands"
#     minimal_client.publisher_ = minimal_client.create_publisher(Float64MultiArray, publish_topic, 1)
#     msg = Float64MultiArray()
    

#     # read packplan
#     packplan, rotation_index = read_csv_packplan()
    
#     # read container from csv and calculate wall thickness of the container
#     c_wall_x, c_wall_y, c_wall_z, container = read_csv_container()
    
#     # calculate corrections due to collisions
#     col_correction, corr_width, corr_length, corr_height = collision_correction(packplan, i = 0)

#     # calculate pick- and place-poses
#     package_pick_poses, package_place_poses = calculate_poses(packplan, col_correction, corr_width, corr_length, corr_height, rotation_index, c_wall_x, c_wall_y, c_wall_z, container)
    
#     print(package_pick_poses)
#     print("&")
#     print(package_place_poses)


#     for i in range(len(packplan)):
        
#         # choose cylinder and tcp
#         index_msg, tcp_cylinder = choose_cylinder(packplan, i)
        
#         # offset over pick- and place-poses for pre- and post-pick-poses
#         pre_post_pick_offset = Affine(translation = [0, 0, (2 * packplan.height[i])], rotation = [0, 0, 0])
#         pre_post_place_offset = Affine(translation = [0, 0, container.height[0] + c_wall_z], rotation = [0, 0, 0])


#         # Pick
#         pre_pick_pose = pre_post_pick_offset * package_pick_poses[i] * tcp_cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(pre_pick_pose), False, True) # move to pre_pick_pose
#         pick_pose = package_pick_poses[i] * tcp_cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(pick_pose), True, True) # move to pick_pose
#         msg.data = msg_cylinder[index_msg]
#         minimal_client.publisher_.publish(msg) # extend cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(pick_pose), True, True) # stay at pick_pose while cylinder extends
#         # attach:
#         req = AttachObject.Request()
#         attached_object = req.attached_object_details
#         index_string = str(index_msg)
#         attached_object.link_name = "tcp_" + index_string
#         attached_object.object.header.frame_id = "base"
#         attached_object.object.id = "box"
#         pose = Pose()
#         pose.position.x = 0.347 + (packplan.width[i] / 2) + 0.0005
#         pose.position.y = 0.911 + (packplan.length[i] / 2) + 0.0005
#         pose.position.z = 0.94 + (packplan.height[i] / 2) 
#         primitive = SolidPrimitive()
#         primitive.type = primitive.BOX
#         primitive.dimensions = [0.0] * 3
#         primitive.dimensions[0] = packplan.width[i]
#         primitive.dimensions[1] = packplan.length[i]
#         primitive.dimensions[2] = packplan.height[i]
    
#         attached_object.object.primitives.insert(len(attached_object.object.primitives),primitive)
#         attached_object.object.primitive_poses.insert(len(attached_object.object.primitive_poses),pose)

#         attached_object.object.operation = attached_object.object.ADD
#         attached_object.touch_links = ["tcp_" + index_string]
    
#         minimal_client.send_attach_request(attached_object, True)
#         # end attach
        
       
#         post_pick_pose = pre_post_pick_offset * package_pick_poses[i] * tcp_cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(post_pick_pose), True, True) # move to post_pick_pose

#         print(pre_post_place_offset)
#         print(package_place_poses[i])
#         print(tcp_cylinder)
#         # Place
#         pre_place_pose = pre_post_place_offset * package_place_poses[i] * tcp_cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(pre_place_pose), False, True) # move to pre_place_pose
#         place_pose = package_place_poses[i] * tcp_cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(place_pose), True, True) # move to place_pose# attach:
        
#         # detach
#         req_detach = DetachObject.Request()
#         detach_object = req_detach.detached_object_details
#         detach_object.object.id = "box"
#         detach_object.link_name = "tcp_" + index_string

#         response_detach = minimal_client.send_detach_request(detach_object, True)
#         # end detach
#         # attach
#         req = AttachObject.Request()
#         attached_object = req.attached_object_details
#         index_string = str(index_msg)
#         attached_object.link_name = "container" 
#         attached_object.object.header.frame_id = "base"
#         attached_object.object.id = "box"
#         pose = Pose()
#         pose.position.x =c_corner_x - c_wall_x - container.length[0] + corr_length + packplan.y[i] + (packplan.parallel_c_width[i] / 2) + 0.0005
#         pose.position.y = c_corner_y + c_wall_y + packplan.x[i] + (packplan.parallel_c_length[i] / 2) + corr_width + 0.0005
#         pose.position.z = c_corner_z + c_wall_z + corr_height + packplan.z[i] + (packplan.height[i] / 2) 
#         pose.orientation.z = 1.0
#         primitive = SolidPrimitive()
#         primitive.type = primitive.BOX
#         primitive.dimensions = [0.0] * 3
#         primitive.dimensions[0] = packplan.width[i]
#         primitive.dimensions[1] = packplan.length[i]
#         primitive.dimensions[2] = packplan.height[i]
        
#         attached_object.object.primitives.insert(len(attached_object.object.primitives),primitive)
#         attached_object.object.primitive_poses.insert(len(attached_object.object.primitive_poses),pose)

#         attached_object.object.operation = attached_object.object.ADD
#         attached_object.touch_links = ["container"]
    
#         minimal_client.send_attach_request(attached_object, True)
#         # end attach
#         msg.data = msg_cylinder[0]
#         minimal_client.publisher_.publish(msg) # retract cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(place_pose), True, True) # stay at place_pose while cylinder retracts
        
#         post_place_pose = pre_post_place_offset * package_place_poses[i] * tcp_cylinder
#         response = minimal_client.send_move_request(affine_to_pose_msg(post_place_pose), True, True) # move to post_place_pose
        

#     minimal_client.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     load_container_cylinder()

