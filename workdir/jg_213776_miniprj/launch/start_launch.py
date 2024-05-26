from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():


    return LaunchDescription([
        Node(
            package='jg_213776_miniprj',
            executable='minipr_node',
            name='minipr_node'
        ),
    ])