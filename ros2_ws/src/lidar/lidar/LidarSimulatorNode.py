import socket
import msgpack
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSimulatorNode(Node):
    def __init__(self):
        self.address = '0.0.0.0'
        self.port = 17001

        super().__init__('lidar_simulator_node')
        self.get_logger().info('Starting node for recieving lidar data from sim and publishing to topic')

        self.lidar_publisher = self.create_publisher(LaserScan, 'simulator/lidar', 10)

        self.receive_lidar_data_and_publish()

    def receive_lidar_data_and_publish(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((self.address, self.port))
            while True:
                resp = s.recvfrom(4096)
                laser_scan = self.convert_msg_to_laser_scan(resp[0])
                self.lidar_publisher.publish(laser_scan)

    def convert_msg_to_laser_scan(self, msg: bytes) -> LaserScan:
        laser_scan = LaserScan()
        scan_dict = msgpack.unpackb(msg, raw=False)

        laser_scan.angle_min = scan_dict['angle_min']
        laser_scan.angle_max = scan_dict['angle_max']
        laser_scan.angle_increment = scan_dict['angle_increment']

        laser_scan.time_increment = scan_dict['time_increment']

        laser_scan.scan_time = scan_dict['scan_time']

        laser_scan.range_min = scan_dict['range_min']
        laser_scan.range_max = scan_dict['range_max']


        laser_scan.ranges = scan_dict['ranges']
        laser_scan.intensities = scan_dict['intensities']

        return laser_scan


def main(args=None):
    rclpy.init(args=args)

    lidar = LidarSimulatorNode()
    rclpy.spin(lidar)
    rclpy.shutdown()

if __name__ == '__main__':
    main()