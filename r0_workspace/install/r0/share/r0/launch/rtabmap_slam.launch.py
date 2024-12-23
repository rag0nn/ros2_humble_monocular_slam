from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

"""
MONOCULAR VERİ İLE SLAM YAPMAK İÇİN LAUNCH DOSYASI (RTABMAP MONOCULAR VERİ KABUL ETMEDİĞİ İÇİN ÇALIŞMIYOR)
MINE
"""
def generate_launch_description():

    return LaunchDescription([


        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'map_frame_id' : 'map',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd':True,
                'rgb_topic': '/rgb/image',
                'memory_path': '/home/user/rtabmap_memory',
                'detect_mapping': True,
                'publish_maps': True,
                'approx_sync':True, # odom is generated from images, so we can exactly sync all inputs
                'map_negative_poses_ignored':True,
                #'subscribe_odom_info': False,
                'OdomF2M/MaxSize': '1000',
                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.00001',
                #'Reg/Strategy': '0',
                'Vis/OdomOnly': True, # Use Visual Odometry only
                'odom_sensor_sync': True,

            },
            ],
            remappings=[
                #('rgb/image', 'camera/image_raw'),
                #('rgb/camera_info', 'camera/info'),
            ]
        ),


    ])