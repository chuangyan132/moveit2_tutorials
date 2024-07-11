from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    z_decrement_arg = DeclareLaunchArgument(
        'z_decrement',
        default_value=EnvironmentVariable('Z_DECREMENT', default_value='0.05'),
        description='Z decrement value'
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="gripper_down",
        package="test_moveit",
        executable="gripper_down",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'z_decrement': LaunchConfiguration('z_decrement')}
        ],
    )

    return LaunchDescription([z_decrement_arg, move_group_demo])
