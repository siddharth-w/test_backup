
import rclpy
from rclpy.node import Node
from squaternion import Quaternion
from geometry_msgs.msg import PoseStamped
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
        # print("yo1")

        self.subscription = self.create_subscription(
            PoseStamped,
            'phantom/pose',
            self.pose_callback,
            0
        )
        # print("yo2")

    def pose_callback(self, msg):
        # print("yo")
        r = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q = Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        # q = Quaternion(1,0,0,0)
        # print("q",q)
        q = q.to_euler(degrees=False)
        _pose = owl_client.Pose()
        # _pose.x = msg.position.x 
        # _pose.y = msg.position.y
        # _pose.z = msg.position.z
        # _pose.roll = msg.orientation.x
        # _pose.pitch = msg.orientation.y
        # _pose.yaw = msg.orientation.z
        _pose.x = -0.0004907564664197173
        # _pose.x = (r[0] + 0.2)*(0.4/0.46) - 0.23

        _pose.y = -0.4047704292723893
        # _pose.y = (r[1] - 0.07)*(-0.16/-0.15) - 0.24

        # _pose.z = (r[2] + 0.1)*(0.39/0.3) + 0.07
        _pose.z = 0.40957452674154166



        # _pose.roll = -2.9720449402046487
        _pose.pitch = 0.01278618507445185
        _pose.yaw = -1.7781526628056141
        # _pose.yaw = q[2]

        _pose.roll = q[0]

        # _pose.pitch = q[2]

        # _pose.roll = q[2]
        # print(_z)
        # print(q)
        # print("e",q)
        # q = Quaternion.from_euler(q[0],q[1],q[2], degrees=True)
        # print(q)
        jointspeed = 50
        toolspeed = 50  # mm/sec
        print("pose: ", _pose.x, _pose.y, _pose.z, _pose.roll, _pose.pitch, _pose.yaw)

        self.client.move_to_pose(_pose, toolspeed, wait=True)
        # self.get_logger().info(f' x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')

        time.sleep(1)

def main(args=None):
    # print("yo3")
    try:
        rclpy.init(args=args)
        node = PoseSubscriberNode()
        # print("yo4")

        rclpy.spin(node)
        # print("yo5")
    except KeyboardInterrupt:

        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
