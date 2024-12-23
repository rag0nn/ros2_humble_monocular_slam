from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_odom',  # RTAB-Map odometri düğümü paketi
            executable='rgbd_odometry',  # Çalıştırılabilir dosya adı
            name='rtabmap_odom',  # Düğüm adı
            output='screen',  # Çıktılar ekran üzerinde gösterilecek
            parameters=[
                {
                    # Odometri tipi: Görsel odometri (Frame-to-Frame)
                    'Odom/Type': 'FrameToFrame',
                    # Görsel özellik çıkarma stratejisi
                    'Odom/Strategy': "0",  # Görsel tabanlı odometri
                    # Harici odometri kullanılmayacak
                    'RGBD/UseOdom': False,
                    # Görsel özellik çıkarma tipi (örneğin, ORB)
                    'Vis/FeatureType': "6",
                    # Minimum iç nokta (inlier) sayısı
                    'Vis/MinInliers': "20",
                    # Dönüşüm tahmini tipi (PnP kullanılıyor)
                    'Vis/EstimationType': "0",
                    # Epipolar geometri doğrulaması aktif
                    'Vis/EpipolarGeometry': True,
                    # Maksimum eşleştirme mesafesi
                    'Vis/MaxDepth': "10.0",
                    # Görüntüdeki maksimum eşleştirme sayısı
                    'Vis/MaxFeatures': "1000",
                    'approx_sync': True,
                    "Odometry/UseTimeSync": True,
                    'Reg/MinFeatures':5,
                    'Kp/DetectorStrategy': 3 ,
                    'odom_frame_id' : 'odom',
                    'sync_queue_size' : 20,
                    'topic_queue_size' : 20


                }
            ],
            remappings=[
            #('rgb/image', 'camera/image_raw'),
            #('depth/image','camera/image_depth'),
            #('rgb/camera_info', 'camera/info'),
            ]
        ),
    ])
