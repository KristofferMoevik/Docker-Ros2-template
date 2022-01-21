import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy import qos
import time


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 'pose_with_covariance_stamped', self.listener_callback, qos.qos_profile_sensor_data)
        self.subscription

    def listener_callback(self, msg):
        print(msg)
            

def main(args=None):
    rclpy.init(args=args)

    subscriber_node = SubscriberNode()

    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()