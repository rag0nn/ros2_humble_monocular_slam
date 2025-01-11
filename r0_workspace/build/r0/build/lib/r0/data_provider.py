import rclpy
import rclpy.node
from sensor_msgs.msg import Image,CameraInfo    
from std_msgs.msg import Header

from .depth_estimation.run import MidasEstimator
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from glob import glob

NODE_NAME = "data_provider"
TOPIC_NAME_PUB_RGB = "rgbd_sync_node/rgb/image"
TOPIC_NAME_PUB_DEPTH = "rgbd_sync_node/depth/image"
TOPIC_NAME_PUB_CAMERA_INFO = "rgbd_sync_node/rgb/camera_info"
IMAGES_PATH = "/home/rag0n/Desktop/data/test_videos/sequence_44/images"
PUBLISH_PERIOT = 3
PUBLISH_QUEUE_SIZE = 30
FRAME_ID = "camera_link"
   
class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.depth_estimator = MidasEstimator()
        self.bridge = CvBridge()

        self.pub_rgb = self.create_publisher(Image,TOPIC_NAME_PUB_RGB,PUBLISH_QUEUE_SIZE)
        self.pub_depth = self.create_publisher(Image,TOPIC_NAME_PUB_DEPTH,PUBLISH_QUEUE_SIZE)
        self.pub_caminfo = self.create_publisher(CameraInfo,TOPIC_NAME_PUB_CAMERA_INFO,PUBLISH_QUEUE_SIZE)

        self.static_cam_info = self._get_camera_info()

        self.counter_seq = 0

        self.images = sorted(glob(os.path.join(IMAGES_PATH, '*.png')) + glob(os.path.join(IMAGES_PATH, '*.jpg')))
        if not self.images: raise RuntimeError("Görüntü dosyası bulunamadı")

        self.get_logger().info("Görüntü akışı başlatılıyor")
        self.timer = self.create_timer(PUBLISH_PERIOT,self.sequence_read_callback)

            
    def sequence_read_callback(self):
        frame = cv2.imread(self.images[self.counter_seq])

        depth_frame = np.float32(cv2.cvtColor(self.depth_estimator.estimate(frame),cv2.COLOR_BGR2GRAY))

        self.counter_seq +=1
        self.publish_callback(frame,depth_frame)

    def publish_callback(self,rgb,depth):
        timestamp = self.get_clock().now().to_msg()

        # rgb
        msg_rgb = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        msg_rgb.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # depth
        msg_depth = self.bridge.cv2_to_imgmsg(depth,encoding='32FC1')
        msg_depth.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # camera info
        self.static_cam_info.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # publish
        self.pub_rgb.publish(msg_rgb)
        self.pub_depth.publish(msg_depth)
        self.pub_caminfo.publish(self.static_cam_info)

        self.get_logger().info(f"Veriler Paylaşıldı:{self.counter_seq-1} timestamp-{timestamp}")

    def _get_camera_info(self):
        msg = CameraInfo()
        msg.width,msg.height = 1280,960
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [0.669566858850269, 0.0, 0.500408664348414,
                 0.0, 0.493248545285398, 0.897966326944875,
                 0.0, 0.0, 1.0
                 ]
        return msg








def main():
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()