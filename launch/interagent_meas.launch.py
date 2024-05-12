from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def generate_launch_description():
    gz_world_name = LaunchConfiguration('gz_world_name')
    gz_model_names = LaunchConfiguration('gz_model_names')
    ros_ns = LaunchConfiguration('ros_ns')

    gz_world_name_arg = DeclareLaunchArgument(
        'gz_world_name',
        default_value='AbuDhabiSwarm'
    )
    gz_model_names_arg = DeclareLaunchArgument(
        'gz_model_names',
        default_value="[x500_1, x500_2, cf1, cf2, cf3]"
    )
    ros_ns_arg = DeclareLaunchArgument(
        'ros_ns',
        default_value="[px4_1, px4_2, cf_1, cf_2, cf_3]"
    )
    
    interagent_meas_publisher = Node(
        package='gz_interagent_meas',
        executable='interagent_range_pub',
        parameters=[
            {'gz_world_name': gz_world_name},
            {'gz_model_names': gz_model_names},
            {'ros_ns': ros_ns},
        ]
    )

    return LaunchDescription([
        gz_world_name_arg,
        gz_model_names_arg,
        ros_ns_arg,
        interagent_meas_publisher,
    ])