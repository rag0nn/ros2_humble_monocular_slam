import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

class OdometryToBaseLinkPublisher(Node):

    def __init__(self):
        super().__init__('odometry_to_base_link_publisher')

        # Transform Broadcaster oluşturuluyor
        self.broadcaster = TransformBroadcaster(self)

        # Odometry mesajlarını alacak bir abone oluşturuluyor
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Odometry verilerini aldığınız topic
            self.odom_callback,
            10  # Mesaj kuyruğu boyutu
        )



    def odom_callback(self, msg: Odometry):
        # Odometry mesajından dönüşüm bilgilerini alıyoruz
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'  # Kaynak çerçevesi
        transform.child_frame_id = 'base_footprint'  # Hedef çerçevesi

        # Sabit öteleme bilgisi
        transform.transform.translation.x = 10.0
        transform.transform.translation.y = 20.0
        transform.transform.translation.z = 15.0

        # Kuaterniyonu alıyoruz
        orientation = msg.pose.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

        # Kuaterniyonun normunu hesaplıyoruz
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)

        # Eğer norm sıfır veya geçersizse, varsayılan bir kuaterniyon kullanın
        if norm == 0 or math.isnan(norm):
            self.get_logger().warn("Geçersiz kuaterniyon tespit edildi! Varsayılan değer atanıyor.")
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            # Kuaterniyonu normalize edin
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        # Normalize edilmiş kuaterniyonu dönüşüme ekleyin
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # Dönüşümü yayınlıyoruz
        self.broadcaster.sendTransform(transform)



def main(args=None):
    rclpy.init(args=args)
    node = OdometryToBaseLinkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
