from Classes.packplan import Packplan
from Classes.container_transfer import container_transfer
from Classes.order_data_transfer import order_data_transfer
import pandas as pd

class Packplan_Processing:

    # Intitialiiere Klasse
    def __init__(self):
        pass

    def read_csv_packplan():
        # read packplan from csv
        # first 3 columns: dimension parralel to container length, heigth, width
        column_names = ['parallel_c_length', 'height', 'parallel_c_width', 'x', 'z', 'y', 'is_stack', 'package_ID', 'package_sequence_nr', 'weight']
        # packplan = pd.read_csv('src/pick_package/Packalgorithmus/Simulationen/' + order + ' BI01 Simulation Packplan.csv', names = column_names)
        
        # Convert the data into a DataFrame
        packplan = pd.DataFrame(Packplan.get_packplan(), columns=column_names)
        
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
                rotation_index.append(1)

            else:
                # orientation 3
                length.append(packplan.parallel_c_width[i])
                width.append(packplan.parallel_c_length[i])
                rotation_index.append(3)

        # insert the length, width, rotation_index, and cylinder as columns into the packplan list
        packplan.insert(10, 'length', length)
        packplan.insert(11, 'width', width)
        packplan.insert(12, 'rotation_index', rotation_index)

        Packplan.set_packplan(packplan)

        return packplan, rotation_index
    

    def read_csv_container():
        
        # read which container is used
        column_names = ['width', 'height', 'length']
        
        # convert mm into m
        container = pd.DataFrame(container_transfer.get_container(), columns=column_names)
        container.width = container.width / 1000
        container.height = container.height / 1000
        container.length = container.length / 1000
        
        # print("Container:\n",container)

        if container.width[0] == 0.585:
            #container special
            container_outside_x = 0.404
            container_inside_x = container.length[0]
            container_outside_y = 0.599
            container_inside_y = container.width[0]
            container_outside_z = 0.200
            container_inside_z = container.height[0]
        else: print("No container with these dimensions")    

        # wall thickness of the container
        c_wall_x = (container_outside_x - container_inside_x) / 2
        c_wall_y = (container_outside_y - container_inside_y) / 2
        c_wall_z = (container_outside_z - container_inside_z)

        return c_wall_x, c_wall_y, c_wall_z, container

    # calculate the coordinates of the place pose (center of the package + Z-direction)
    def cal_place_coordinates():
        packplan = pd.DataFrame(Packplan.get_packplan())
        x = []
        y = []
        z = []
        for i in range(len(packplan)):
            x.append(packplan.x[i] + packplan.length[i] / 2)
            y.append(packplan.y[i] + packplan.width[i] / 2)
            z.append(packplan.z[i] + packplan.height[i])

        # insert the x, y, z coordinates as columns into the packplan list
        packplan.insert(13, 'x_pack', x)
        packplan.insert(14, 'y_pack', y)
        packplan.insert(15, 'z_pack', z)

        Packplan.set_packplan(packplan)

        return packplan

    # Füge Label ODTF in Packplan ein
    def add_label_odtf():
        packplan = pd.DataFrame(Packplan.get_packplan())
        order_data = order_data_transfer.get_order_data()
        
        label_odtf = []

        # Subtrahiere 1 von package_ID, damit Liste bei 0 anfängt
        packplan["package_ID"] = packplan["package_ID"] - 1

        # Füge die Label ODTF entsprechend der package_ID ein
        for i in range(len(packplan)):
            for j in range(len(order_data)):
                if packplan["package_ID"][i] == j:
                    label_odtf.append(order_data[j][0])

        packplan.insert(16, 'label_odtf', label_odtf)

        Packplan.set_packplan(packplan)

        return packplan

    # Nicht notwendige Spalten aus Packplan entfernen
    def clear_packplan():
        packplan = pd.DataFrame(Packplan.get_packplan())
        packplan = packplan.drop(columns=['package_sequence_nr', 'parallel_c_length', 'parallel_c_width', 'x', 'y', 'z', 'is_stack', 'package_ID'])
        
        # Ändere Reihenfolge der Spalten
        packplan = packplan[['label_odtf', 'length', 'width', 'height', 'weight', 'rotation_index', 'x_pack', 'y_pack', 'z_pack']]
        
        Packplan.set_packplan(packplan)

        return packplan


    def generate_output():
        
        print("\nStart Pack Processing\n")
        
        # read packplan
        packplan, rotation_index = Packplan_Processing.read_csv_packplan()
        
        # read container from csv and calculate wall thickness of the container
        c_wall_x, c_wall_y, c_wall_z, container = Packplan_Processing.read_csv_container()

        # index_msgs, tcps_cylinder = Pack_Algorithm.choose_cylinder(packplan)
        packplan = Packplan_Processing.cal_place_coordinates()
        packplan = Packplan_Processing.add_label_odtf()
        packplan = Packplan_Processing.clear_packplan()

        print("\nPackplan:")
        print(packplan)

        print("\nContainer:")
        print(container)
    
        print("\n### Finished ###\n")
        return packplan, container
    
# if __name__ == '__main__':
#     final_packplan, final_container = Packplan_Processing.generate_output()