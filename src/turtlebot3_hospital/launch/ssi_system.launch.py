from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    task_file_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_hospital'),
        'config',
        'tasks.yaml'
    ])
    return LaunchDescription([
        Node(
            package='turtlebot3_hospital',
            executable='ssi_auctioneer',
            name='ssi_auctioneer',
            output='screen'
        ),
        Node(
            package='turtlebot3_hospital',
            executable='task_generator',
            name='task_generator',
            parameters=[{'task_file': task_file_path}],
            output='screen'
        ),
    ])
