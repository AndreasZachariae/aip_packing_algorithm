import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/docker/ros2_ws/src/pkg_pack_node/pkg_pack_node/install/control_node'
