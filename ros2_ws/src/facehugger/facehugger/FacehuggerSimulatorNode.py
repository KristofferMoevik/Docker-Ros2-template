import socket
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class FacehuggerSimulatorNode(Node):
    def __init__(self):
        self.address = '10.24.39.95'
        self.port = 17000

        # init node
        super().__init__('facehugger_simulator_node')
        self.get_logger().info('Starting node for sending instructions to the facehugger in simulator')

        # create move forward service
        self.move_forward_srv = self.create_service(Trigger, 'facehugger/move_forward', self.move_forward)
        self.get_logger().info('Creating /facehugger/move_forward service')

        # create detach service
        self.detach_srv = self.create_service(Trigger, 'facehugger/detach', self.detach)
        self.get_logger().info('Creating /facehugger/detach service')
    
    def move_forward(self, request, response):
        resp = self.send_tcp_message_to_simulator_facehugger(b'move forward')
        response.success = resp == b'success'
        return response

    def detach(self, request, response):
        resp = self.send_tcp_message_to_simulator_facehugger(b'detach')
        response.success = resp == b'success'
        return response

    def send_tcp_message_to_simulator_facehugger(self, message: bytes) -> bytes:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.address, self.port))
                s.send(message)
                return s.recv(1024)
            except:
                return b'failure'

def main(args=None):
    rclpy.init(args=args)

    facehugger = FacehuggerSimulatorNode()

    rclpy.spin(facehugger)
   
    rclpy.shutdown()

if __name__ == '__main__':
    main()
