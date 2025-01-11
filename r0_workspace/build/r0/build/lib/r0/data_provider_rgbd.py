
import rclpy
import rclpy.node
from sensor_msgs.msg import Image,CameraInfo    
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from glob import glob
import time

NODE_NAME = "data_provider"
TOPIC_NAME_PUB_RGB = "camera/rgb/image_color"
TOPIC_NAME_PUB_DEPTH = "camera/depth/image"
TOPIC_NAME_PUB_CAMERA_INFO = "camera/rgb/camera_info"
IMAGES_PATH = "/home/rag0n/Desktop/data/data/rgbd_dataset_freiburg2_pioneer_slam2/rgb_sync"
DEPTH_IMAGES_PATH = "/home/rag0n/Desktop/data/data/rgbd_dataset_freiburg2_pioneer_slam2/depth_sync"
PUBLISH_PERIOT = 0.33
PUBLISH_QUEUE_SIZE = 30
FRAME_ID = "kinect"

class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rgb_names = []
        self.depth_names = []
        self._get_image_names()
        self.bridge = CvBridge()

        self.pub_rgb = self.create_publisher(Image,TOPIC_NAME_PUB_RGB,PUBLISH_QUEUE_SIZE)
        self.pub_depth = self.create_publisher(Image,TOPIC_NAME_PUB_DEPTH,PUBLISH_QUEUE_SIZE)
        self.pub_caminfo = self.create_publisher(CameraInfo,TOPIC_NAME_PUB_CAMERA_INFO,PUBLISH_QUEUE_SIZE)

        self.static_cam_info = self._get_camera_info()

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence starting with {PUBLISH_PERIOT} second periot...")
        self.timer = self.create_timer(PUBLISH_PERIOT,self.publish_callback)

            
    def sequence_read_callback(self):
        frame = cv2.imread(self.rgb_names[self.counter_seq])
        depth_frame = cv2.imread(self.depth_names[self.counter_seq],cv2.IMREAD_UNCHANGED)
        print(f"\nIMAGE RGB: {self.rgb_names[self.counter_seq]} SHAPE: ",frame.shape)
        print(f"IMAGE DEPTH: {self.depth_names[self.counter_seq]} SHAPE: ",depth_frame.shape)

        self.counter_seq +=1
        return frame,depth_frame

    def publish_callback(self):
        start_callback_time = time.time()
        timestamp = self.get_clock().now().to_msg()
        rgb,depth = self.sequence_read_callback()

        # rgb
        msg_rgb = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
        msg_rgb.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # depth
        msg_depth = self.bridge.cv2_to_imgmsg(depth,encoding='16UC1')
        msg_depth.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # camera info
        self.static_cam_info.header = Header(stamp=timestamp,frame_id=FRAME_ID)

        # publish
        self.pub_rgb.publish(msg_rgb)
        self.pub_depth.publish(msg_depth)
        self.pub_caminfo.publish(self.static_cam_info)

        end_callback_time = time.time()
        callback_duration = end_callback_time - start_callback_time
        self.get_logger().info(f"Data Published: {self.counter_seq-1}  timestamp-{timestamp}")
        self.get_logger().info(f"Total read and publish time: {callback_duration}")

    def _get_camera_info(self):
        msg = CameraInfo()
        msg.width,msg.height = 1280,960
        msg.distortion_model = "plumb_bob"
        msg.d = [0.2312,-0.7849,-0.0033,-0.0001,0.9172]
        msg.k = [520.9, 0.0, 325.1,
                 0.0, 521.0, 249.7,
                 0.0, 0.0, 1.0
                 ]
        return msg
    
    def _get_image_names(self):

        rgb_names = sorted(glob(f"{IMAGES_PATH}/*"))        
        depth_names = sorted(glob(f"{DEPTH_IMAGES_PATH}/*"))
        self.rgb_names = rgb_names
        self.depth_names = depth_names









def main():
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
