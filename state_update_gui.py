#!/usr/bin/env python
import rclpy
# from rclpy.qos import qos_profile_default
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing to Float64MultiArray!
---------------------------
Control Joints:
   q : Joint 0 +0.1
   a : Joint 0 -0.1
   w : Joint 1 +0.1
   s : Joint 1 -0.1
   e : Joint 2 +0.1
   d : Joint 2 -0.1
   r : Joint 3 +0.1
   f : Joint 3 -0.1
   t : Joint 4 +0.1
   g : Joint 4 -0.1
   y : Joint 5 +0.1
   h : Joint 5 -0.1

CTRL-C to quit
"""

jointBindings = {
    'q': (0, 0.1),
    'a': (0, -0.1),
    'w': (1, 0.1),
    's': (1, -0.1),
    'e': (2, 0.1),
    'd': (2, -0.1),
    'r': (3, 0.1),
    'f': (3, -0.1),
    't': (4, 0.1),
    'g': (4, -0.1),
    'y': (5, 0.1),
    'h': (5, -0.1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):    
    if args is None:
        args = sys.argv

    rclpy.init()
    node = rclpy.create_node('teleop_joint_keyboard')
    
    pub = node.create_publisher(Float64MultiArray, 'robot/joint_state_update', 1)

    # Initial joint positions (assuming 6 joints, initialize all to 0.0)
    joint_positions = [0.0] * 6

    try:
        msg = Float64MultiArray()
        while True:
            key = getKey()
            if key in jointBindings.keys():
                joint_index, delta = jointBindings[key]
                # Update the joint position
                joint_positions[joint_index] += delta
                # Create and publish the Float64MultiArray message
                msg.data = joint_positions
                pub.publish(msg)
                print(f"Joint positions: {joint_positions}")
            elif key == '\x03':  # CTRL-C to quit
                break

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        # Send zero positions to stop the robot
        joint_positions = [0.0] * 6
        msg = Float64MultiArray()
        msg.data = joint_positions
        pub.publish(msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
