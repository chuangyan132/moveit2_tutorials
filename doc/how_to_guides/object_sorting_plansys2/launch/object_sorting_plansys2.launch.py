import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    work_dir = get_package_share_directory('object_sorting_plansys2')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py'
        )),
        launch_arguments={
            'model_file':
            work_dir + '/pddl/object_sorting.pddl'
        }.items()
    )

    move_to_pose_cmd = Node(
        package='object_sorting_plansys2',
        executable='move_to_pose_action',
        name='move_to_pose_action',
        output='screen',
        parameters=[])

    pick_cmd = Node(
        package='object_sorting_plansys2',
        executable='pick_action',
        name='pick_action',
        output='screen',
        parameters=[])

    place_cmd = Node(
        package='object_sorting_plansys2',
        executable='place_action',
        name='place_action',
        output='screen',
        parameters=[])

    ld = LaunchDescription()

    ld.add_action(plansys2_cmd)
    ld.add_action(move_to_pose_cmd)
    ld.add_action(pick_cmd)
    ld.add_action(place_cmd)

    return ld