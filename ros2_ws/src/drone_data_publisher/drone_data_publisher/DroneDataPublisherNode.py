import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from pymavlink import mavutil
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

class PublisherNode(Node):
    syst = None
    comp = None
    master = None
    global_pos_msg = None
    attitude_msg = None

    def command_long_send_await_ack(self,
        target_system,
        target_component,
        command_id,
        param1 = 0,
        param2 = 0,
        param3 = 0,
        param4 = 0,
        param5 = 0,
        param6 = 0,
        param7 = 0,
        timeout = 0.1,
        retries = 30 # max: 255
    ):
        ack_msg = None
        i = 0
        while ack_msg == None:
            self.master.mav.command_long_send(target_system, target_component,
                        command_id, i, param1, param2, param3, param4, param5, param6, param7)
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
            i += 1
            if(i > retries):
                break
        if(ack_msg == None or ack_msg.to_dict()["result"] != 0):
            print("Command sending failed.", ack_msg)
        return ack_msg

    def debug_pos(self):
        # sets interval for specified command [microseconds]. Ardupilot SITL Struggles with shorter interval than 1e6/300 (300 Hz)
        ack_msg_global_pos = self.command_long_send_await_ack(self.syst, self.comp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1 = mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                param2 = 1e6/1000)
        ack_msg_attitude = self.command_long_send_await_ack(self.syst, self.comp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1 = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                param2 = 1e6/1000)

        t = time.time()
        n = 0
        while 1:
            publishdata = False
            msg = self.master.recv_match(
                type = 'GLOBAL_POSITION_INT', blocking=False # LOCAL_POSITION_NED alternative
            )
            if msg is not None:
                self.global_pos_msg = msg
                publishdata = True

            msg = self.master.recv_match(
                type = 'ATTITUDE_QUATERNION', blocking=False
            )
            if msg is not None:
                self.attitude_msg = msg
                publishdata = True
            
            if publishdata and self.global_pos_msg is not None and self.attitude_msg is not None:
                pose_with_covariance = PoseWithCovarianceStamped()
                
                pose_with_covariance.pose.pose.position.x = float(self.global_pos_msg.to_dict()["lat"])
                pose_with_covariance.pose.pose.position.y = float(self.global_pos_msg.to_dict()["lon"])
                pose_with_covariance.pose.pose.position.z = float(self.global_pos_msg.to_dict()["alt"])

                pose_with_covariance.pose.pose.orientation.x = float(self.attitude_msg.to_dict()["q2"])
                pose_with_covariance.pose.pose.orientation.y = float(-self.attitude_msg.to_dict()["q4"])
                pose_with_covariance.pose.pose.orientation.z = float(self.attitude_msg.to_dict()["q3"])
                pose_with_covariance.pose.pose.orientation.w = float(self.attitude_msg.to_dict()["q1"])

                pose_with_covariance.pose.covariance = [0.0] * 36
                pose_with_covariance.header.stamp = self.get_clock().now().to_msg()

                self.publisher.publish(pose_with_covariance)

    def __init__(self):
        super().__init__('publisher') # Initialize node with name 
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose_with_covariance_stamped', 10) #Create a publisher

        # Create a connection to the drone; save boot time
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
        print("Waiting for heartbeat ...")
        self.master.wait_heartbeat()
        print("It's alive")
        self.syst = self.master.target_system
        self.comp = self.master.target_component

        self.debug_pos()

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
