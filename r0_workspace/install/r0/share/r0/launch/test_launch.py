
# Example to run rgbd datasets:
# [ROS1] Prepare ROS1 rosbag for conversion to ROS2
#   $ wget http://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.bag
#   $ rosbag decompress rgbd_dataset_freiburg3_long_office_household.bag
#   $ wget https://raw.githubusercontent.com/srv/srv_tools/kinetic/bag_tools/scripts/change_frame_id.py
#   Edit change_frame_id.py, remove/comment lines beginning with "PKG" and "import roslib", change line "Exception, e" to "Exception"
#   $ roscore
#   $ python3 change_frame_id.py -o rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag -i rgbd_dataset_freiburg3_long_office_household.bag -f openni_rgb_optical_frame -t /camera/rgb/image_color
# [ROS2]
#   $ sudo pip install rosbags     # See https://docs.openvins.com/dev-ros1-to-ros2.html
#   $ rosbags-convert rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag

#   $ ros2 launch rtabmap_examples rgbdslam_datasets.launch.py
#   $ cd rgbd_dataset_freiburg3_long_office_household_frameid_fixed
#   $ ros2 bag play rgbd_dataset_freiburg3_long_office_household_frameid_fixed.db3 --clock

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, SetRemap
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'subscribe_rgbd':True,
          'approx_sync':False, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored':True,
          'subscribe_odom_info': True,
          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '1000',
          'GFTT/MinDistance': '10',
          'GFTT/QualityLevel': '0.00001',
          #'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          #'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings=[
         ('rgbd_sync_node/rgb/image','rgb/image'),
         ('rgbd_sync_node/depth/image','depth/image'),
         ('rgbd_sync_node/rgb/camera_info','rgb/camera_info'),
         ('rgbd_image', '/rgbd_camera/rgbd_image'),
         ('odom','/vo')]
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='true',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),
        # Synchronize stereo data together in a single topic
        # Issue: stereo_img_proc doesn't produce color and 
        #        grayscale images exactly the same (there is a small 
        #        vertical shift with color), we should use grayscale for 
        #        left and right images to get similar results than on ros1 noetic.
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            namespace='rgbd_sync_node',
            remappings=remappings
            ),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[parameters],
            remappings=remappings),
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
    ])