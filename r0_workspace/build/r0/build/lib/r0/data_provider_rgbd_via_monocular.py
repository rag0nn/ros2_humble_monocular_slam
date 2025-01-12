from .depth_estimation.run import MidasEstimator

import rclpy
import rclpy.node
from sensor_msgs.msg import Image,CameraInfo    
from std_msgs.msg import Header


from cv_bridge import CvBridge
import cv2
import numpy as np

from glob import glob
import time



NODE_NAME = ""
TOPIC_NAME_PUB_RGB = ""
TOPIC_NAME_PUB_DEPTH = ""
TOPIC_NAME_PUB_CAMERA_INFO = ""
IMAGES_PATH = ""
PUBLISH_PERIOT ="" 
PUBLISH_QUEUE_SIZE = ""
FRAME_ID = ""
CAM_WIDTH,CAM_HEIGHT = 0,0
CAM_D,CAM_D_MATRIX,CAM_K_MATRIX = "",[],[]





class DataProvider(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.rgb_names = []
        self._get_image_names()

        self.depth_estimator = MidasEstimator()
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
        depth_frame = cv2.cvtColor(self.depth_estimator.estimate(frame), cv2.COLOR_BGR2GRAY).astype(np.uint16)
        print(f"\nIMAGE RGB: {self.rgb_names[self.counter_seq]} SHAPE: ",frame.shape)
        print(f"IMAGE DEPTH: {self.rgb_names[self.counter_seq]} SHAPE: ",depth_frame.shape)

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
        msg.width,msg.height = CAM_WIDTH,CAM_HEIGHT
        msg.distortion_model = CAM_D
        msg.d = CAM_D_MATRIX
        msg.k = CAM_K_MATRIX
        return msg
    
    def _get_image_names(self):
        rgb_names = sorted(glob(f"{IMAGES_PATH}/*"))        
        self.rgb_names = rgb_names








def get_params():
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        import yaml

        global NODE_NAME, TOPIC_NAME_PUB_RGB, TOPIC_NAME_PUB_DEPTH, TOPIC_NAME_PUB_CAMERA_INFO
        global IMAGES_PATH, PUBLISH_PERIOT, PUBLISH_QUEUE_SIZE, FRAME_ID
        global CAM_D,CAM_D_MATRIX,CAM_K_MATRIX,CAM_WIDTH,CAM_HEIGHT

        package_name = 'r0'
        path = os.path.join(
            get_package_share_directory(package_name), 'config', 'const_params.yaml'
        )
        # YAML dosyasını oku
        with open(path, "r") as file:
            data = yaml.safe_load(file)

        NODE_NAME = data["node_names"]["data_provider"]
        TOPIC_NAME_PUB_RGB = data["topics"]["image_rgb"]
        TOPIC_NAME_PUB_DEPTH = data["topics"]["image_depth"]
        TOPIC_NAME_PUB_CAMERA_INFO = data["topics"]["camera_info"]
        IMAGES_PATH = data["data_read"]["rgb_path"]
        PUBLISH_PERIOT = data["publish_config"]["publish_periot"]
        PUBLISH_QUEUE_SIZE = data["publish_config"]["publish_queue_size"]
        FRAME_ID = data["publish_config"]["frame_id"]

        CAM_WIDTH =data["data_read"]["camera_info"]["resolution"]["width"]
        CAM_HEIGHT = data["data_read"]["camera_info"]["resolution"]["height"]
        CAM_D =data["data_read"]["camera_info"]["distortion_model"]
        CAM_D_MATRIX = data["data_read"]["camera_info"]["distortion_coefficients"]
        CAM_K_MATRIX = data["data_read"]["camera_info"]["intrinsic_matrix"]

    except:
        raise Exception("Params not loaded successfully!!!")


def main():
    get_params()
    rclpy.init()
    node = DataProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()

