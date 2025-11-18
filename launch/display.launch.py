from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('porsche')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

    default_model_path = os.path.join(pkg_share, 'src','description','porsche_description.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    world_path = os.path.join(pkg_share, 'world', 'wall_world.sdf')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_path = os.path.join(pkg_share, 'world', 'maps','my_map_save.yaml')
    slam_path = os.path.join(pkg_share,'config','mapper_params_online_async.yaml')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'porsche',
            'z': '0.65',
        }.items(),
    )
    
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    #Nav2 Nodes
    
    nav2_config = RewrittenYaml(
        source_file=nav2_params_path,
        root_key='',
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True
        
    )
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_config,
            'slam': 'True',   # True = SLAM Toolbox, False = AMCL + map
            'map': '',
            'map_server':'False',
            'mapper_params_file': slam_path
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        ekf,
        
        #nav2_bringup
    ])
