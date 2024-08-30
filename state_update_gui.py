#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt
import math
from threading import Lock, Thread

class JointControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        # Initialize ROS 2
        rclpy.init()
        self.node = rclpy.create_node('gui_joint_control')
        self.pub = self.node.create_publisher(Float64MultiArray, 'robot/joint_states', 10)
        self.subscriber = JointStatesSubscriber(self.node, self.update_joint_angles_from_ros)

        # Initialize joint angles
        self.joint_angles = [0.0] * 9
        self.lock = Lock()

        # GUI elements
        self.init_ui()

        # Start ROS 2 spinning in a separate thread
        self.ros_thread = Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

    def init_ui(self):
        self.setWindowTitle('Joint Control GUI')

        # Create layout
        layout = QVBoxLayout()

        # Define maximum slider value for 2π (approx 6.2832 radians)
        max_slider_value = 1000  # Scaled value for precision

        # Create sliders for each joint
        self.sliders = []
        self.labels = []

        for i in range(9):
            label = QLabel(f'Joint {i+1} Angle (radians):')
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, max_slider_value)  # Range from 0 to 1000
            slider.setValue(int(self.joint_angles[i] * (max_slider_value / (2 * math.pi))))  # Set initial value

            slider.valueChanged.connect(lambda value, index=i: self.update_joint_angle(index, value))

            self.labels.append(label)
            self.sliders.append(slider)
            layout.addWidget(label)
            layout.addWidget(slider)

        # Create arrow buttons for joint control
        arrow_buttons_layout = QVBoxLayout()

        self.arrow_buttons = []
        self.arrow_labels = []

        for i in range(9):
            button_layout = QHBoxLayout()

            increment_button = QPushButton(f'Joint {i+1} +')
            decrement_button = QPushButton(f'Joint {i+1} -')

            increment_button.clicked.connect(lambda _, index=i: self.adjust_joint_angle(index, 0.1))
            decrement_button.clicked.connect(lambda _, index=i: self.adjust_joint_angle(index, -0.1))

            button_layout.addWidget(increment_button)
            button_layout.addWidget(decrement_button)

            arrow_buttons_layout.addLayout(button_layout)

        layout.addLayout(arrow_buttons_layout)

        self.setLayout(layout)

    def update_joint_angle(self, joint_index, value):
        # Convert slider value to radians
        max_slider_value = 1000
        self.joint_angles[joint_index] = (value / max_slider_value) * (2 * math.pi)  # Convert to radians
        
        # Automatically publish updated joint states
        self.publish_joint_states()

    def adjust_joint_angle(self, joint_index, delta):
        with self.lock:
            # Adjust the joint angle by delta radians
            self.joint_angles[joint_index] += delta
            # Ensure joint angles are within 0 to 2π radians
            self.joint_angles[joint_index] = max(0, min(self.joint_angles[joint_index], 2 * math.pi))
            # Update the slider position
            slider_value = int(self.joint_angles[joint_index] * (1000 / (2 * math.pi)))
            self.sliders[joint_index].setValue(slider_value)
            # Publish updated joint states
            self.publish_joint_states()

    def publish_joint_states(self):
        # Create and publish the message
        joint_msg = Float64MultiArray()
        joint_msg.data = self.joint_angles
        self.pub.publish(joint_msg)

    def update_joint_angles_from_ros(self, joint_angles):
        with self.lock:
            self.joint_angles = joint_angles
            for i in range(min(len(self.sliders), len(joint_angles))):
                # Update slider value based on the current joint angle
                self.sliders[i].setValue(int(joint_angles[i] * (1000 / (2 * math.pi))))

    def closeEvent(self, event):
        # Clean up ROS 2 resources
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

class JointStatesSubscriber(Node):
    def __init__(self, parent_node, update_callback):
        super().__init__('joint_states_subscriber')
        self.update_callback = update_callback
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'robot/joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Process the received message and update the GUI
        joint_angles = msg.data
        self.update_callback(joint_angles)

def main():
    app = QApplication([])
    gui = JointControlGUI()
    gui.show()
    app.exec_()

if __name__ == '__main__':
    main()
