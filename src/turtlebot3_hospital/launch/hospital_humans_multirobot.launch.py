#!/usr/bin/env python3

import os
from os import path, environ, pathsep

from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                                   PythonExpression, EnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from controller_manager.launch_utils import generate_load_controller_launch_description

def generate_launch_description():

    # Configuracion inicial

    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    navgoal_topic = LaunchConfiguration('navgoal_topic')
    ignore_models = LaunchConfiguration('ignore_models')
    navigation = LaunchConfiguration('navigation')


    # Launch Humanos

    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'scenarios',
        LaunchConfiguration('configuration_file')
    ])

    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    )

    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'hospital.world'
    ])

    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
                    {'use_gazebo_obs': gz_obs},
                    {'update_rate': rate},
                    {'robot_name': robot_name},
                    {'global_frame_to_publish': global_frame},
                    {'use_navgoal_to_start': use_navgoal},
                    {'navgoal_topic': navgoal_topic},
                    {'ignore_models': ignore_models}]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])
    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    # Modelos y recursos Gazebo

    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
    ])
    aws_hospital_models = PathJoinSubstitution([
        FindPackageShare('aws_robomaker_hospital_world'),
        'models'
    ])
    multi_robot_models = PathJoinSubstitution([
        FindPackageShare('turtlebot3_multi_robot'),
        'models'
    ])

    model, plugin, media = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    set_env_gazebo_model = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=['/opt/ros/humble/share/turtlebot3_gazebo/models', ':', my_gazebo_models, ':', 
               aws_hospital_models, ':', multi_robot_models, ':', '/usr/share/gazebo-11/models']
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
    )

    # Launch Gazebo

    config_file = path.join(get_package_share_directory('hunav_gazebo_wrapper'), 'launch', 'params.yaml')
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world'
    ])

    gzserver_process = ExecuteProcess(
        cmd=[
            'gzserver', world_path,
            _boolean_command('verbose'), '',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '--ros-args',
            '--params-file', config_file
        ],
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )

    gzclient_process = ExecuteProcess(
        cmd=['gzclient', _boolean_command('verbose'), ' '],
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )

    gz_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[gzserver_process, gzclient_process],
                )
            ]
        )
    )



    static_tf_node = Node(
        package="tf2_ros", 
        executable="static_transform_publisher",
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(navigation)
    )

    # Launch robots y Nav2

    robots = [
        {'name': 'tb1', 'x_pose': '9.0', 'y_pose': '15.0', 'z_pose': '0.01'},
        {'name': 'tb2', 'x_pose': '8.0', 'y_pose': '15.0', 'z_pose': '0.01'},
        {'name': 'tb3', 'x_pose': '7.0', 'y_pose': '15.0', 'z_pose': '0.01'},
    ]
    TURTLEBOT3_MODEL = 'burger'
    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')
    hospital_robot = get_package_share_directory('turtlebot3_hospital')
    nav_launch_dir = os.path.join(turtlebot3_multi_robot, 'launch', 'nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    params_file = LaunchConfiguration('nav_params_file')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(hospital_robot, 'config', 'hospital_map.yaml')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    multi_robot_spawn_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver_process,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=_generate_multi_robot_actions(
                        robots, TURTLEBOT3_MODEL, turtlebot3_multi_robot, nav_launch_dir,
                        use_sim_time, params_file, rviz_config_file, enable_drive, enable_rviz
                    ) + [map_server, map_server_lifecycle]
                )
            ]
        )
    )


    # Planificador de trayectorias humanos

    human_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_hospital'),
                'launch',
                'human_planner.launch.py' 
            )
        )
    )


    planner_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver_process,
            on_start=[
                TimerAction(
                    period=6.0, 
                    actions=[human_planner_launch]
                )
            ]
        )
    )

    # Launch Planificador de tareas y generador

    auctioneer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_hospital'),
                'launch',
                'ssi_system.launch.py'
            )
        )
    )

    auctioneer_event = RegisterEventHandler(
        OnProcessStart(
            target_action=gzserver_process,
            on_start=[
                TimerAction(
                    period=25.0,  
                    actions=[auctioneer_launch]
                )
            ]
        )
    )

    # Marcadores humanos y robots, solo para visualización en Rviz

    human_markers_node = Node(
        package='turtlebot3_hospital',
        executable='human_markers_pub',
        name='human_markers_pub',
        output='screen'
    )

    rviz_config_path = os.path.join(hospital_robot, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )



    # Launch 

    ld = LaunchDescription()

    # Variables y argumentos
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)

    ld.add_action(DeclareLaunchArgument('configuration_file', default_value='agents_hospital.yaml'))
    ld.add_action(DeclareLaunchArgument('metrics_file', default_value='metrics.yaml'))
    ld.add_action(DeclareLaunchArgument('base_world', default_value='hospital.world'))
    ld.add_action(DeclareLaunchArgument('use_gazebo_obs', default_value='True'))
    ld.add_action(DeclareLaunchArgument('update_rate', default_value='100.0'))
    ld.add_action(DeclareLaunchArgument('robot_name', default_value='tb1'))
    ld.add_action(DeclareLaunchArgument('global_frame_to_publish', default_value='map'))
    ld.add_action(DeclareLaunchArgument('use_navgoal_to_start', default_value='False'))
    ld.add_action(DeclareLaunchArgument('navgoal_topic', default_value='goal_pose'))
    ld.add_action(DeclareLaunchArgument('navigation', default_value='False'))
    ld.add_action(DeclareLaunchArgument('ignore_models', default_value='ground_plane cafe'))
    ld.add_action(DeclareLaunchArgument('verbose', default_value='False'))
    ld.add_action(DeclareLaunchArgument('robot_namespace', default_value=''))

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_drive', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz_config_file',
        default_value=os.path.join(turtlebot3_multi_robot, 'rviz', 'multi_nav2_default_view.rviz')))
    ld.add_action(DeclareLaunchArgument('nav_params_file',
        default_value=os.path.join(hospital_robot, 'config', 'nav2_params.yaml')))

    # Humanos y Gazebo

    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)
    ld.add_action(hunav_manager_node)
    ld.add_action(hunav_evaluator_node)
    ld.add_action(gz_launch_event)
    ld.add_action(static_tf_node)
    ld.add_action(rviz_node)

    # Robots

    ld.add_action(multi_robot_spawn_event)
    
    # Planificador humano
    
    ld.add_action(planner_event)
    
    # Marcadores para Rviz
    
    ld.add_action(human_markers_node)

    #Planificador de tareas

    ld.add_action(auctioneer_event)


    return ld

# Funciones auxiliares

def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    return PythonExpression(cmd)

def _generate_multi_robot_actions(robots, model, multi_robot_pkg, nav_launch_dir,
                                  use_sim_time, params_file, rviz_config_file,
                                  enable_drive, enable_rviz):
    actions = []
    urdf = os.path.join(multi_robot_pkg, 'urdf', f'turtlebot3_{model}.urdf')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    last_action = None
    for robot in robots:
        namespace = '/' + robot['name']
        state_pub = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf]
        )
        spawn_tb3 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(multi_robot_pkg, 'models', f'turtlebot3_{model}', 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', robot['z_pose'], '-Y', '0.0', '-unpause'
            ],
            output='screen'
        )
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'namespace': namespace,
                'use_namespace': 'True',
                'map': '',
                'map_server': 'False',
                'params_file': params_file,
                'default_bt_xml_filename': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                'autostart': 'true',
                'use_sim_time': use_sim_time, 'log_level': 'warn'
            }.items()
        )




        if last_action is None:
            actions.extend([state_pub, spawn_tb3, bringup])
        else:
            actions.append(
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=last_action,
                        on_start=[spawn_tb3, state_pub, bringup]
                    )
                )
            )
        last_action = spawn_tb3

    for robot in robots:
        namespace = '/' + robot['name']
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
                robot['x_pose'] + ', y: ' + robot['y_pose'] + \
                ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '5', '--qos-reliability', 'reliable', namespace + '/initialpose',
                 'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace,
                              'use_namespace': 'True',
                              'rviz_config': rviz_config_file,
                              'log_level': 'warn'}.items(),
            condition=IfCondition(enable_rviz)
        )
        drive_tb3 = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            namespace=namespace,
            name='smoother_server',
            output='screen',
            parameters=[params_file],
        )
        actions.extend([initial_pose_cmd, rviz_cmd, drive_tb3])
        actions.append(
            TimerAction(
            period=2.0,
            actions=[smoother_server]
        )
)
    return actions

