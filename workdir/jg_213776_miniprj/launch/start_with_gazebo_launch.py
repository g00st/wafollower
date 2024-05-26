from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the turtlebot3_gazebo package
    turtlebot3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        Node(
            package='jg_213776_miniprj',
            executable='minipr_node',
            name='minipr_node'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_path, '/launch/turtlebot3_house.launch.py']),
            launch_arguments=[('x_pose', '-1.4'), ('y_pose', '3.5')]
        )
    ])