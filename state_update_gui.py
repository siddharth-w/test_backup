import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
import threading

class JointStatesSubscriber(Node):
    def __init__(self, gui):
        super().__init__('joint_states_subscriber')
        self.gui = gui
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'robot/joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Update GUI with the received joint states
        joint_angles = msg.data
        self.gui.update_joint_angles(joint_angles)

class JointControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.joint_angles = [0.0] * 9
        self.lock = threading.Lock()

        # Initialize ROS 2
        rclpy.init()
        self.node = rclpy.create_node('gui_joint_control')
        self.subscriber = JointStatesSubscriber(self)
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

    def init_ui(self):
        self.setWindowTitle('Joint Control GUI')

        # Create layout
        layout = QVBoxLayout()

        # Create labels for each joint angle
        self.labels = []
        for i in range(9):
            label = QLabel(f'Joint {i+1} Angle (radians): 0.0')
            self.labels.append(label)
            layout.addWidget(label)

        self.setLayout(layout)

    def update_joint_angles(self, joint_angles):
        with self.lock:
            self.joint_angles = joint_angles
            for i in range(min(len(self.labels), len(joint_angles))):
                self.labels[i].setText(f'Joint {i+1} Angle (radians): {joint_angles[i]:.2f}')

    def closeEvent(self, event):
        # Clean up ROS 2 resources
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication([])
    gui = JointControlGUI()
    gui.show()
    app.exec_()

if __name__ == '__main__':
    main()
