# Move simulated robot

first terminal:
1. ./start_docker.sh
2. colcon build
3. source install/setup.bash
4. launch rviz + moveit_wrapper (together in one launch-file):
   1. no container:
          ros2 launch kuka_kr10r1100sixx_cell_description demo_modified.launch.py
          not required:
          (robot_description_package:=kuka_kr10r1100sixx_cell_description robot_description_file:=kr10_cylinder.xacro semantic_description_file:=kr10_cylinder.srdf)
   2. with container (replace container_name):
          ros2 launch kuka_kr10r1100sixx_cell_description demo_modified.launch.py robot_description_package:=kuka_kr10r1100sixx_cell_description robot_description_file:=kr10_container_<container_name>.xacro semantic_description_file:=kr10_cylinder.srdf
   

second Terminal:
1. docker exec -it <docker_name> /bin/bash
2. run program(replace order_name in src -> pick_package -> pick_package -> examples.py):
   1. ros2 run pick_package pack_container
   


order_name	container_name
______________________________
Beispiel_1	x03
Beispiel_2	x01
Beispiel_3	x07
Beispiel_4	x02
Beispiel_5	x04
Beispiel_6	x05
