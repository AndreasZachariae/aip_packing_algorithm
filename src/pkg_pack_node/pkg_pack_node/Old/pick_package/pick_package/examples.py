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

# test window in RViz, if desired: True
testWindow = False

# order name
order = 'Beispiel_1'


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
    # packplan.weight = packplan.weight / 1000

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
        str_i = str(i)
        if packplan.width[i] <= 0.051 and packplan.weight[i] < 0.09:
            index_msg = 3
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [-0.075, 0.106, 0.281], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.051 and packplan.width[i] <= 0.092 and packplan.length[i] > 0.051 and packplan.length[i] <= 0.129 and packplan.weight[i] < 1.14:
            index_msg = 1
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [0.0485, 0.0815, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.051 and packplan.width[i] < 0.151 and packplan.length[i] > 0.129 and packplan.length[i] <= 0.232 and packplan.weight[i] < 2.27:
            index_msg = 2
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.035 and packplan.width[i] <= 0.151 and packplan.length[i] > 0.165 and packplan.length[i] <= 0.232 and packplan.weight[i] < 3.10:
            index_msg = 4 
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [-0.0585, -0.055, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.053 and packplan.width[i] <= 0.182 and packplan.length[i] > 0.232 and packplan.weight[i] < 3.41:
            index_msg = 5
            index_msgs.append(index_msg)
            tcp_cylinder =  Affine(translation = [0.049, -0.009, 0.299], rotation=[0.0, 0.0, 0.0])
            tcps_cylinder.append(tcp_cylinder)
        elif packplan.width[i] > 0.151:
            if packplan.length[i] <= 0.165 and packplan.weight[i] < 2.27:
                index_msg = 2
                index_msgs.append(index_msg)
                tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)
            elif packplan.length[i] > 0.165 and packplan.length[i] <= 0.243 and packplan.weight[i] < 5.37:
                index_msg = 6
                index_msgs.append(index_msg)
                tcp_cylinder =  Affine(translation = [-0.001, -0.054, 0.299], rotation=[0.0, 0.0, 0.0]) 
                tcps_cylinder.append(tcp_cylinder)
            elif packplan.length[i] > 0.243 and packplan.weight[i] < 6.51:
                index_msg = 7
                index_msgs.append(index_msg)
                tcp_cylinder =  Affine(translation = [-0.001, -0.015, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)
            else: print("Error: dimensions of package " + str_i + " not allowable")
        else: print("Error: dimensions of package " + str_i + " not allowable")

    return index_msgs, tcps_cylinder
    



def correct_collision(packplan, container, index_msgs, rotation_index):
    # calculation of the gripper overhang
    # correction of the place_pose due to the gripper overhang
    single_corr_x = []
    single_corr_y = []
    # check if there is enough space due to the gripper overhang
    remaining_space_x = []
    remaining_space_y = []
    for i in range(len(packplan)):
        if index_msgs[i] == 1:
            # cylinder back left
            if rotation_index[i] == 0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.086:
                    correction_x = (0.086 - packplan.width[i]) / 2
                else: correction_x = 0.0
                if packplan.length[i] < 0.056:
                    correction_y = (0.056 - packplan.length[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.width[i] < 0.078:
                    space_x = (0.078 - packplan.width[i]) / 2
                else: space_x = 0.0
                if packplan.length[i] < 0.132:
                    space_y = (0.132 - packplan.length[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_x = 0.139 - (packplan.width[i] / 2)
                    space_y = 0.217 - (packplan.length[i] / 2)
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:
                    correction_y = correction_y + 0.01 # cylinder front left bigger than cylinder back left
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                if packplan.length[i] < 0.132:
                    correction_x = (0.132 - packplan.length[i]) / 2
                else: correction_x = 0.0
                if packplan.width[i] < 0.086:
                    correction_y = (0.086 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                space_x = 0.0
                if packplan.width[i] < 0.078:
                    space_y = (0.078 - packplan.width[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    corr_x = - (packplan.length[i] / 2) - packplan.y[i] + 0.217
                    corr_y = - (packplan.width[i] / 2) - packplan.x[i] + 0.139
                    if corr_x > 0:
                        correction_x = correction_x + 0.217 - max(0.066, (packplan.length[i] / 2))
                    if corr_y > 0:
                        correction_y = correction_y + 0.139 - max(0.039, (packplan.width[i] / 2))
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:
                    space_x = space_x + 0.01
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)


        elif index_msgs[i] == 2:
            # cylinder back right
            if rotation_index[i] == 0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.072:
                    correction_x = (0.072 - packplan.width[i]) / 2
                else: correction_x = 0.0
                correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.width[i] < 0.092:
                    space_x = (0.092 - packplan.width[i]) / 2
                else: space_x = 0.0
                space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_x = 0.141 - (packplan.width[i] / 2)
                    corr_y = - (packplan.length[i] / 2) - packplan.x[i] + 0.177
                    if corr_y > 0:
                        correction_y = correction_y + 0.177 - max(0.0645, (packplan.length[i] / 2))
                    space_y = space_y + 0.011 # cylinder front right bigger than cylinder back right
                    correction_x = correction_x + 0.005 # cylinder back left bigger than cylinder back right
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                correction_x = 0.0
                if packplan.width[i] < 0.072:
                    correction_y = (0.072 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                space_x = 0.0
                if packplan.width[i] < 0.092:
                    space_y = (0.092 - packplan.width[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_x = 0.177 - (packplan.length[i] / 2)
                    space_y = 0.141 - (packplan.width[i] / 2)
                    correction_x = correction_x + 0.011 # cylinder front right bigger than cylinder back right
                    correction_y = correction_y + 0.005 # cylinder back left bigger than cylinder back right
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)

        elif index_msgs[i] == 3:
            # cylinder front left
            if rotation_index[i] == 0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.132:
                    correction_x = (0.132 - packplan.width[i]) / 2
                else: correction_x = 0.0
                if packplan.length[i] < 0.022:
                    correction_y = (0.022 - packplan.length[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.width[i] < 0.032:
                    space_x = (0.032 - packplan.width[i]) / 2
                else: space_x = 0.0
                if packplan.length[i] < 0.172:
                    space_y = (0.172 - packplan.length[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] - 0.018 > gripper_extension:
                    corr_x = - (packplan.width[i] / 2) - packplan.y[i] + 0.166
                    if corr_x > 0:
                        correction_x = correction_x + 0.166 - max(0.066, (packplan.width[i] / 2))
                    space_y = 0.242 - (packplan.length[i] / 2)
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                if packplan.length[i] < 0.166:
                    correction_x = (0.166 - packplan.length[i]) / 2
                else: correction_x = 0.0
                if packplan.width[i] < 0.132:
                    correction_y = (0.132 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.length[i] < 0.022:
                    space_x = (0.022 - packplan.length[i]) / 2
                else: space_x = 0.0
                if packplan.length[i] < 0.032:
                    space_y = (0.032 - packplan.length[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] - 0.018 > gripper_extension:
                    corr_x = - (packplan.length[i] / 2) - packplan.y[i] + 0.242
                    corr_y = - (packplan.width[i] / 2) - packplan.x[i] + 0.166
                    if corr_x > 0:
                        correction_x = correction_x + 0.243 - max(0.083, (packplan.length[i] / 2)) + 0.003
                    if corr_y > 0:
                        correction_y = correction_y + 0.166 - max(0.066, (packplan.width[i] / 2))
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)

        elif index_msgs[i] == 4:
            # cylinder front right
            if rotation_index[i] ==0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.1:
                    correction_x = (0.1 - packplan.width[i]) / 2
                else: correction_x = 0.0
                correction_y = 0.0
                # calculation of the gripper overhang for the space checking 
                if packplan.width[i] < 0.066:
                    space_x = (0.066 - packplan.width[i]) / 2
                else: space_x = 0.0
                space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    corr_x = - (packplan.width[i] / 2) - packplan.y[i] + 0.099
                    corr_y = - (packplan.length[i] / 2) - packplan.x[i] + 0.0875
                    if corr_x > 0:
                        correction_x = correction_x + corr_x
                    if corr_y > 0:
                        correction_y = correction_y + corr_y
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                correction_x = 0.0
                if packplan.width[i] < 0.1:
                    correction_y = (0.1 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking 
                space_x = 0.0
                if packplan.width[i] < 0.066:
                    space_y = (0.066 - packplan.width[i]) / 2
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_x = 0.17 - (packplan.length[i] / 2)
                    corr_y = - (packplan.width[i] / 2) - packplan.x[i] + 0.149
                    if corr_y > 0:
                        correction_y = correction_x + 0.149 - max(0.05, (packplan.width[i] / 2))
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)

        elif index_msgs[i] == 5:
            # cylinder back right, cylinder back left
            if rotation_index[i] == 0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.084:
                    correction_x = (0.084 - packplan.width[i]) / 2 
                else: correction_x = 0.0
                correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.width[i] < 0.106:
                    space_x = (0.106 - packplan.width[i]) / 2
                else: space_x = 0.0
                space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_x = 0.14 - (packplan.width[i] / 2)
                    space_y = space_y + 0.011 # cylinder front right bigger than cylinder back right
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:    
                    correction_y = correction_y + 0.01 #cylinder front left bigger than cylinder back left
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                correction_x = 0
                if packplan.width[i] < 0.084:
                    correction_y = (0.084 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                space_x = 0.0
                if packplan.width[i] < 0.09:
                    space_y = (0.09 - packplan.width[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_y = 0.14 - (packplan.width[i] / 2)
                    correction_x = correction_x + 0.011 # cylinder front right bigger than cylinder back right
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:    
                    space_x = space_x + 0.01 #cylinder front left bigger than cylinder back left
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)

        elif index_msgs[i] == 6:
            # cylinder back right, cylinder front right
            if rotation_index[i] == 0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.172:
                    correction_x = (0.172 - packplan.width[i]) / 2
                else: correction_x = 0.0
                correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.width[i] < 0.182:
                    space_x = (0.182 - packplan.width[i]) / 2
                else: space_x = 0.0
                space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    corr_y = - (packplan.length[i] / 2) - packplan.x[i] + 0.17
                    if corr_y > 0:
                        correction_y = correction_y + 0.17 - (packplan.length[i] / 2)
                    correction_x = correction_x + 0.005 # cylinder back left bigger than cylinder back right
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                correction_x = 0.0
                if packplan.width[i] < 0.172:
                    correction_y = (0.172 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                space_x = 0.0
                if packplan.width[i] < 0.182:
                    space_y = (0.182 - packplan.width[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] > gripper_extension:
                    space_x = 0.17 - (packplan.length[i] / 2)
                    correction_y = correction_y + 0.005 # cylinder back left bigger than cylinder back right
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)

        elif index_msgs[i] == 7:
            # cylinder back left, cylinder back right, cylinder front right
            if rotation_index[i] == 0:
                # calculation of the gripper overhang for the correction
                if packplan.width[i] < 0.182:
                    correction_x = (0.182 - packplan.width[i]) / 2
                else: correction_x = 0.0
                correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                if packplan.width[i] < 0.182:
                    space_x = (0.182 - packplan.width[i]) / 2
                else: space_x = 0.0
                space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension: 
                    corr_y = - (packplan.length[i] / 2) - packplan.x[i] + 0.131
                    if corr_y > 0:
                        correction_y = correction_y + 0.131 - (packplan.length[i] / 2)
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:    
                    correction_y = correction_y + 0.01 #cylinder front left bigger than cylinder back left
            if rotation_index[i] == 1:
                # calculation of the gripper overhang for the correction
                correction_x = 0.0
                if packplan.width[i] < 0.182:
                    correction_y = (0.182 - packplan.width[i]) / 2
                else: correction_y = 0.0
                # calculation of the gripper overhang for the space checking
                space_x = 0.0
                if packplan.width[i] < 0.182:
                    space_y = (0.182 - packplan.width[i]) / 2
                else: space_y = 0.0
                # if the height of the package is not enough, the not extended gripper may collide with the container
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:
                    space_x = 0.131 - (packplan.length[i] / 2)
                if container.height[0] - packplan.z[i] - packplan.height[i] + 0.018 > gripper_extension:    
                    space_x = space_x + 0.01 #cylinder front left bigger than cylinder back left
            single_corr_x.append(correction_x)
            single_corr_y.append(correction_y)
            remaining_space_x.append(space_x)
            remaining_space_y.append(space_y)
            

    # calculation of the total correction
    total_corr_x = []
    total_corr_y = []

    # add first package
    total_corr_x.append(single_corr_x[0])
    total_corr_y.append(single_corr_y[0])
    

    for i in range(1, len(packplan)):
        if packplan.y[i] == 0:
            total_x = single_corr_x[i]
        else: 
            total_x = total_corr_x[0] + single_corr_x[i]
        if packplan.x[i] == 0:
            total_y = single_corr_y[i]
        else:
            total_y = total_corr_y[i-1] + single_corr_y[i]
        total_corr_x.append(total_x)
        total_corr_y.append(total_y)

    
    return total_corr_x, total_corr_y, remaining_space_x, remaining_space_y
    




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
    total_corr_x, total_corr_y, remaining_space_x, remaining_space_y = correct_collision(packplan, container, index_msgs, rotation_index)
    

    # calculate pick- and place-poses
    package_pick_poses, package_place_poses = calculate_poses(packplan, total_corr_x, total_corr_y, rotation_index, c_wall_x, c_wall_y, c_wall_z, container)
    
    # calculate attach-poses for attaching the package to the gripper and to the container
    attach_position_x, attach_position_y, attach_position_z, detach_position_x, detach_position_y, detach_position_z = calculate_attach_poses(packplan, package_pick_poses, package_place_poses)

    

    for i in range(len(packplan)):
        
        if ((packplan.parallel_c_width[i] + packplan.y[i] + total_corr_x[i] + remaining_space_x[i]) < container.length[0]) and ((packplan.parallel_c_length[i] + packplan.x[i] + total_corr_y[i] + remaining_space_y[i]) < container.width[0]):
            
            # offset over pick- and place-poses for pre- and post-pick-poses
            pre_post_pick_offset = Affine(translation = [0, 0, container.height[0] + 0.05], rotation = [0, 0, 0])
            pre_post_place_offset = Affine(translation = [0, 0, container.height[0] + 0.15], rotation = [0, 0, 0])
            
            
            # Pick
            pre_pick_pose = pre_post_pick_offset * package_pick_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(pre_pick_pose), False, testWindow) # move to pre_pick_pose
            pick_pose = package_pick_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(pick_pose), True, testWindow) # move to pick_pose
            msg.data = msg_cylinder[index_msgs[i]]
            minimal_client.publisher_.publish(msg) # extend cylinder
            response = minimal_client.send_move_request(affine_to_pose_msg(pick_pose), True, testWindow) # stay at pick_pose while cylinder extends
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
            minimal_client.send_attach_request(attached_object, testWindow)
            # end attach
            post_pick_pose = pre_post_pick_offset * package_pick_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(post_pick_pose), True, testWindow) # move to post_pick_pose
            
            # Place
            pre_place_pose = pre_post_place_offset * package_place_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(pre_place_pose), False, testWindow) # move to pre_place_pose
            place_pose = package_place_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(place_pose), True, testWindow) # move to place_pose
            # detach package from gripper
            req_detach = DetachObject.Request()
            detach_object = req_detach.detached_object_details
            detach_object.object.id = "box_" + i_string
            detach_object.link_name = "tcp_" + index_string
            response_detach = minimal_client.send_detach_request(detach_object, testWindow)
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
            minimal_client.send_attach_request(attached_object, testWindow)
            # end attach
            msg.data = msg_cylinder[0]
            minimal_client.publisher_.publish(msg) # retract cylinder
            response = minimal_client.send_move_request(affine_to_pose_msg(place_pose), True, testWindow) # stay at place_pose while cylinder retracts
        
            post_place_pose = pre_post_place_offset * package_place_poses[i] * tcps_cylinder[i]
            response = minimal_client.send_move_request(affine_to_pose_msg(post_place_pose), True, testWindow) # move to post_place_pose
        
        else: print("not enough space")
    
 
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    load_container_cylinder()

