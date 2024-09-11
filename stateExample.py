from owl_client import OwlClient
import time

_robot_ip = "10.42.0.3"

client = OwlClient(_robot_ip)

# Wait for Robot to be available to operate
while not client.is_running():
    time.sleep(0.2)

while True:
    # print("Current joint pose", client.get_joint().get_joints(), "\n")
    print("Current TCP pose", client.get_tcp().get_pose())
    time.sleep(0.2)
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose, PoseStamped
# from std_msgs.msg import Float64MultiArray
# from owl_client import OwlClient
# import time

# class RobotStatePublisher(Node):
#     def __init__(self):
#         super().__init__('robot_state_publisher')
#         self.declare_parameter('robot_ip', '10.42.0.3')
#         robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

#         self.client = OwlClient(robot_ip)
        
#         # Wait for Robot to be available to operate
#         while not self.client.is_running():
#             time.sleep(0.2)

#         # Publishers
#         self.joint_pub = self.create_publisher(Float64MultiArray, 'robot/joint_states', 10)
#         self.tcp_pub = self.create_publisher(PoseStamped, 'robot/tcp_pose', 10)
        
#         # Timer to call the update function periodically
#         self.timer = self.create_timer(0.01, self.timer_callback)

#     def timer_callback(self):
#         joint_msg = Float64MultiArray()
#         joint_msg.data = self.client.get_joint().get_joints()

#         # tcp_msg = PoseStamped()
#         # pose = self.client.get_tcp().get_pose()
#         # tcp_msg.header.stamp = self.get_clock().now().to_msg()
#         # tcp_msg.header.frame_id = 'base_link'
#         # tcp_msg.pose.position.x = pose[0]
#         # tcp_msg.pose.position.y = pose[1]
#         # tcp_msg.pose.position.z = pose[2]
#         # tcp_msg.pose.orientation.x = pose[3]
#         # tcp_msg.pose.orientation.y = pose[4]
#         # tcp_msg.pose.orientation.z = pose[5]
#         # tcp_msg.pose.orientation.w = pose[6]

#         self.joint_pub.publish(joint_msg)
#         # self.tcp_pub.publish(tcp_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotStatePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
