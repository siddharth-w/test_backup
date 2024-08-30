import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from owl_client import OwlClient, Joint
import owl_client
import time

class MoveJointNode(Node):
    def __init__(self):
        super().__init__('move_joint_node')

        self._robot_ip = "10.42.0.3"
        self.client = OwlClient(self._robot_ip)

        self.jointSpeed = 10 #degree/sec
        
        while not self.client.is_running():
            time.sleep(0.2)
            self.get_logger().info("Waiting for robot to be available...")

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'robot/joint_state_update',
            self.move_callback,
            1
        )

    def move_callback(self, msg):

        
        #zero configuration
        zero_position = Joint()
        zero_position.Base = msg.data[0]
        zero_position.Shoulder = msg.data[1]
        zero_position.Elbow  = msg.data[2]
        zero_position.Wrist1 = msg.data[3]
        zero_position.Wrist2 = msg.data[4]
        zero_position.Wrist3 = msg.data[5]

        self.get_logger().info(f'Moving to pose: {zero_position.Base}')
        self.client.move_to_joint(zero_position, self.jointSpeed)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = MoveJointNode()
    rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()



