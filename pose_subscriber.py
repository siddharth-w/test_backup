
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from owl_client import OwlClient
import owl_client
import time

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')

        self._robot_ip = "10.42.0.3"
        self.client = OwlClient(self._robot_ip)

        while not self.client.is_running():
            time.sleep(0.2)
            self.get_logger().info("Waiting for robot to be available...")

        self.subscription = self.create_subscription(
            Pose,
            'robot_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):

        _pose = owl_client.Pose()
        _pose.x = msg.position.x 
        _pose.y = msg.position.y
        _pose.z = msg.position.z
        _pose.roll = msg.orientation.x
        _pose.pitch = msg.orientation.y
        _pose.yaw = msg.orientation.z
        toolspeed = 100  # mm/sec

        self.client.move_to_pose(_pose, toolspeed)
        self.get_logger().info(f'Moving to pose: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
