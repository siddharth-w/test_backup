#!/usr/bin/env python
import rclpy
from rclpy.qos import qos_profile_default
from std_msgs.msg import Float64MultiArray

import sys
import select
import termios
import tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing to Float64MultiArray!
---------------------------
Press any key to publish [0.1, 0.1, 0.1, 0.1, 0.1, 0.1] for each joint.
CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args)
    node = rclpy.create_node('float64_multiarray_publisher')

    pub = node.create_publisher(Float64MultiArray, 'root_joint_state_update', qos_profile_default)

    try:
        print(msg)
        while True:
            key = getKey()

            if key:
                float64_array_msg = Float64MultiArray()
                float64_array_msg.data = [0.1] * 6

                pub.publish(float64_array_msg)

            if key == '\x03':  # CTRL-C
                break

    except KeyboardInterrupt:
        pass

    finally:
        # Ensure joints are in neutral position upon exit
        float64_array_msg = Float64MultiArray()
        float64_array_msg.data = [0.0] * 6
        pub.publish(float64_array_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
