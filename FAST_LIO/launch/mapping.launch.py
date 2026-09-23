import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    map_file_path = LaunchConfiguration('map_file_path')
    pct_map_file_path = LaunchConfiguration('pct_map_file_path')
    pct_max_range = LaunchConfiguration('pct_max_range')
    registration_max_range = LaunchConfiguration('registration_max_range')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='rs_fairy.yaml',
        description='Config file'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', default_value='rs_fairy_map.pcd',
        description='PCD output path used by the map_save service'
    )
    declare_pct_map_file_path_cmd = DeclareLaunchArgument(
        'pct_map_file_path', default_value='',
        description='Optional near-field PCD output used to build the PCT map'
    )
    declare_pct_max_range_cmd = DeclareLaunchArgument(
        'pct_max_range', default_value='0.0',
        description='Per-frame maximum range retained in the PCT source PCD'
    )
    declare_registration_max_range_cmd = DeclareLaunchArgument(
        'registration_max_range', default_value='15.0',
        description='Maximum point range used by scan-to-map registration'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                {'use_sim_time': use_sim_time,
                 'map_file_path': ParameterValue(map_file_path, value_type=str),
                 'pct_map_file_path': ParameterValue(
                     pct_map_file_path, value_type=str),
                 'pcd_save.pct_max_range': ParameterValue(
                     pct_max_range, value_type=float),
                 'mapping.registration_max_range': ParameterValue(
                     registration_max_range, value_type=float)}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_pct_map_file_path_cmd)
    ld.add_action(declare_pct_max_range_cmd)
    ld.add_action(declare_registration_max_range_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
