from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    data_provider_node = Node(
        package='r0',
        executable='data_provider',
        name='data_provider')
    
    rtabmap_odom_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('r0'),
                        'launch',
                        'rtabmap_rgbd_odom.launch.py'
                    ])
                ]),
            )

    rtabmap_slam_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('r0'),
                        'launch',
                        'rtabmap_slam.launch.py'
                    ])
                ]),
            ) 
    
    rgbd_sync_node =  Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='sync_rgbd'
        )
    
    static_tf_publisher_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_camera',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'camera_link'],  # Örnek pozisyon ve oryantasyon
        )
    
    dynamic_tf_publisher_node = Node(
        package='r0',
        executable='tf_odomtobaselink',
        name='dynmaic_odom_to_baselink'
    )

    return LaunchDescription([
        rtabmap_odom_launch,
        rtabmap_slam_launch,
        data_provider_node,
        #rgbd_sync_node,
        static_tf_publisher_node,
        dynamic_tf_publisher_node,
    ])
"""
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_camera',
    arguments=['0', '0', '0.5', '0', '0', '0', 'base_footprint', 'camera_link'],  # Örnek pozisyon ve oryantasyon
),
Node(
package='r0_slam',
executable='tfnode',
name='tfnode')
"""