from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='spotrobot_control',
             executable='robot_controller_gazebo.py',
             output='screen'
             ),
        # Node(package='spotrobot_control',
        #      executable='ramped_joypad.py',
        #      output='screen'
        #      ),
     #    Node(package='spotrobot_control',
     #         executable='keyboard_teleop.py',
     #         output='screen'
     #         )
    ])