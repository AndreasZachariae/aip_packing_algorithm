from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("kuka_kr10r1100sixx_cell_description"),
            "config",
            "gripper_publisher.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="control_node",
                executable="publisher_gripper_controller",
                name="publisher_gripper_controller",
                parameters=[position_goals],
                output="both",
                remappings=[("/position_commands", "/gripper_controller/commands")]
            )
        ]
    )