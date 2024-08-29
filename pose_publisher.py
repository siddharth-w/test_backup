# file: my_robot_package/pose_publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')

        # Create a publisher for Pose messages
        self.pose_publisher = self.create_publisher(Pose, 'robot_pose', 10)

        while rclpy.ok():
            self.move()
            time.sleep(3)

    def move(self):
        print("Moving to Pose1")
        # Pose 1
        _pose = Pose()
        _pose.position.x = -0.176
        _pose.position.y = -0.240204
        _pose.position.z = 0.489203
        _pose.orientation.x = 3.1376
        _pose.orientation.y = -0.087288
        _pose.orientation.z = 1.56449
        _pose.orientation.w = 0.0
        # Move to Pose1
        self.pose_publisher.publish(_pose)
        self.get_logger().info(f'Publishing pose: x={_pose.position.x}, y={_pose.position.y}, z={_pose.position.z}')
        
        
        time.sleep(3)
        
        print("Moving to Pose2")
        # Pose 2
        _pose = Pose()
        _pose.position.x = -0.176
        _pose.position.y = -0.240204
        _pose.position.z = 0.159684
        _pose.orientation.x  = 3.1376
        _pose.orientation.y = -0.087288
        _pose.orientation.z  = 1.56449
        _pose.orientation.w = 0.0

        # Move to Pose2
        self.pose_publisher.publish(_pose)
        self.get_logger().info(f'Publishing pose: x={_pose.position.x}, y={_pose.position.y}, z={_pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('!!FINISH!!')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

