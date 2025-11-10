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


'''
Idag: 
-----------------------------------------------------------------------------------------------------------------------------------
Jag gjort så att det går att updatera kartan:
I Terminalen:

1: ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
2: ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

I Rviz:

1: Add Panel -> Map
I map: Välj antingen Global_costmp eller Local_costmap med fixed frame på map.
Hitta markers:
Sätt fixed frame på Odom och marker genom by Topic
-----------------------------------------------------------------------------------------------------------------------------------

Imorgon:
-----------------------------------------------------------------------------------------------------------------------------------
Skapa ett rum genom att använda tutorialen given av Articulated Robotics i hur man skapar sin egen värld på Gazebo.
Använd Det rummet som har skapats på Gazebo för att mappa rummet med SLAM.
MHA: Skapade kartan, implementera en planner så att det går att se en skapad rutt. --> Våran planner är:
                                https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner

Hitta även ett sätt att kontrollera bilen istället för att skriva rakt in i terminalen. Kolla om det går att ändra själva filen så att
topicen som den skrivs i matchar det vi har i cmd/vel. Y'know.

------------------------------------------------------------------------------------------------------------------------------------

I framtiden:
-----------------------------------------------------------------------------------------------------------------------------------
Hitta en controller som styr själva bilen.
Blås hela Pi:en
Fixa Docker så att processeringen går på datorn istället för Pi:en
Överför så att det funkar på bilen genom att kolla på vilka topics som controllern publishar/Subscribar till -> Konvertera till bil
-----------------------------------------------------------------------------------------------------------------------------------
'''




def generate_launch_description():
    pkg_share = get_package_share_directory('porsche')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

    default_model_path = os.path.join(pkg_share, 'src','description','porsche_description.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    
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

    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
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
        ekf_node,
        #odom_tf,
    ])
