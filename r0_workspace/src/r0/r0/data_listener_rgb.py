import rclpy
import rclpy.node
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge
import time

NODE_NAME = ""
TOPIC_NAME_PUB_RGB = ""
LISTEN_PERIOT = None
MEAN_PASSED_TIME_FRAME_COUNT = 10

class DataListener(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()

        self.rgb_sub = self.create_subscription(
            Image,
            TOPIC_NAME_PUB_RGB,
            self.listen_callback,
            LISTEN_PERIOT
        )

        self.counter_seq = 0

        self.get_logger().info(f"Image sequence listening with {LISTEN_PERIOT} listen periot...")


    def listen_callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f"\nDATA:{self.counter_seq} stamp: {msg.header.stamp}")
            
            cv_image = self._print_info(cv_image)

            cv2.imshow("RGB Image", cv_image)
            cv2.waitKey(1)
            self.counter_seq += 1
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

        if self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 0:
            if self.start_time is not None:
                end_time = time.time()
                mean_time = (end_time - self.start_time) / MEAN_PASSED_TIME_FRAME_COUNT
                self.get_logger().info(f"MEAN PASSED TIME PER DATA: {mean_time:.6f} seconds")
            else:
                self.get_logger().warning("Start time is None, skipping mean time calculation.")
        elif self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 1:
            self.start_time = time.time()

    def _print_info(self,image):
        cv2.putText(image,f"{self.counter_seq}",(20,20),cv2.FONT_HERSHEY_DUPLEX,1,(255,0,0),1)
        return image



def get_params():
    try:

        from ament_index_python.packages import get_package_share_directory
        import os
        import yaml

        global NODE_NAME,TOPIC_NAME_PUB_RGB,LISTEN_PERIOT

        package_name = 'r0'
        path = os.path.join(
            get_package_share_directory(package_name), 'config', 'const_params.yaml'
        )
        # YAML dosyasını oku
        with open(path, "r") as file:
            data = yaml.safe_load(file)
        NODE_NAME = data["node_names"]["data_listener_rgb"]
        TOPIC_NAME_PUB_RGB = data["topics"]["image_rgb"]
        LISTEN_PERIOT = data["subscription_config"]["rgb_sub_periot"]   
    except:
        raise Exception("Params not loaded successfully!")


def main():
    get_params()
    rclpy.init()
    node = DataListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()