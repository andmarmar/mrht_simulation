from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    num_humans = 4  # Numero de humanos, necesario cambiar
    
    for hid in range(1, num_humans+1):
        ld.add_action(
            Node(
                package='turtlebot3_hospital',  
                executable='human_planner', 
                name=f'human_planner_{hid}', 
                output='screen',
                parameters=[{'human_id': hid}]
            )
        )


    return ld
