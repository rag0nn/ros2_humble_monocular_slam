import rclpy
import rclpy.node
from sensor_msgs.msg import CameraInfo
import time

NODE_NAME = ""
TOPIC_NAME_PUB_INFO = ""
LISTEN_PERIOT = 0
MEAN_PASSED_TIME_FRAME_COUNT = 10

class DataListener(rclpy.node.Node):
    
    def __init__(self):
        super().__init__(NODE_NAME)

        self.info_sub = self.create_subscription(
            CameraInfo,
            TOPIC_NAME_PUB_INFO,
            self.listen_callback,
            LISTEN_PERIOT
        )

        self.counter_seq = 0

        self.get_logger().info(f"Camera info listening with {LISTEN_PERIOT} listen periot...")


    def listen_callback(self,msg):

        self.get_logger().info(f"\n\nDATA: {self.counter_seq} stamp: {msg.header.stamp}")
        self.get_logger().info(f"Received CameraInfo message:")
        self.get_logger().info(f"Width: {msg.width}, Height: {msg.height}")
        self.get_logger().info(f"Distortion Model: {msg.distortion_model}")
        self.get_logger().info(f"Distortion Coefficients: {msg.d}")
        self.get_logger().info(f"Intrinsic Matrix (K): {msg.k}")
        self.get_logger().info(f"Rectification Matrix (R): {msg.r}")
        self.get_logger().info(f"Projection Matrix (P): {msg.p}")
        self.counter_seq += 1

        if self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 0:
            if self.start_time is not None:
                end_time = time.time()
                mean_time = (end_time - self.start_time) / MEAN_PASSED_TIME_FRAME_COUNT
                self.get_logger().info(f"MEAN PASSED TIME PER DATA: {mean_time:.6f} seconds")
            else:
                self.get_logger().warning("Start time is None, skipping mean time calculation.")
        elif self.counter_seq % MEAN_PASSED_TIME_FRAME_COUNT == 1:
            self.start_time = time.time()



def get_params():
    try:
        from ament_index_python.packages import get_package_share_directory
        import os
        import yaml

        global NODE_NAME,TOPIC_NAME_PUB_INFO,LISTEN_PERIOT

        package_name = 'r0'
        path = os.path.join(
            get_package_share_directory(package_name), 'config', 'const_params.yaml'
        )
        # YAML dosyasını oku
        with open(path, "r") as file:
            data = yaml.safe_load(file)
        NODE_NAME = data["node_names"]["data_listener_caminfo"]
        TOPIC_NAME_PUB_INFO = data["topics"]["camera_info"]
        LISTEN_PERIOT = data["subscription_config"]["cam_info_sub_periot"]
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