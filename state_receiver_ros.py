import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointStatesSubscriber(Node):
    def __init__(self):
        super().__init__('joint_states_subscriber')
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'robot/joint_states',
            self.listener_callback,
            10
        )
        

    def listener_callback(self, msg):
        # Process the received message
        self.get_logger().info(f'Received joint states: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesSubscriber()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
