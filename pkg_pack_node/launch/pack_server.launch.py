import launch

import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pkg_pack_node',
            executable='PackAlgorithm_Server.py',
            name='pack_server',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()