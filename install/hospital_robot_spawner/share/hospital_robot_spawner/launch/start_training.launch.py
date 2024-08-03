import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

# def generate_launch_description():

#     ld = LaunchDescription()

#     training_parameters = os.path.join(
#         get_package_share_directory('hospital_robot_spawner'),
#         'config',
#         'training_parameters.yaml'
#     )

#     start_training = Node(
#         package='hospital_robot_spawner',
#         executable='start_training',
#         #name='hospitalbot_training',
#         parameters=[training_parameters]
#     )

#     ld.add_action(start_training)

#     return ld

def generate_launch_description():

    node_list = [
        ExecuteProcess(
            cmd=['xterm', '-hold','-e', 'ros2', 'run', 'hospital_robot_spawner', 'start_training'],
            output='screen'
        ),
    ]

    return LaunchDescription(node_list)