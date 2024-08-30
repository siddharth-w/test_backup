import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
import sys
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, 'robot/joint_update', 10)
        self.joint_angles = [0.0] * 9  # Initialize joint angles

    def publish_joint_angles(self):
        msg = Float64MultiArray()
        msg.data = self.joint_angles
        self.publisher.publish(msg)

    def set_joint_angles(self, angles):
        self.joint_angles = angles

class JointControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Joint Control GUI')
        self.setGeometry(100, 100, 400, 600)  # Set window size and position
        self.init_ui()

        # Initialize ROS 2
        rclpy.init()
        self.node = rclpy.create_node('joint_control_gui_node')

        # Create a publisher node
        self.publisher = JointPublisher()
        
        # Timer to process ROS 2 callbacks
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)  # Adjust the timer interval as needed (in milliseconds)

    def init_ui(self):
        # Create layout
        layout = QVBoxLayout()

        # Create sliders and buttons for each joint angle
        self.sliders = []
        self.labels = []
        self.increment_buttons = []
        self.decrement_buttons = []

        for i in range(9):
            # Create label and slider
            slider_label = QLabel(f'Joint {i+1} Angle (radians): 0.0')
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(90)  # Center slider at 0 radians
            slider.setTickInterval(10)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(lambda value, idx=i: self.update_joint_angle(value, idx))
            
            # Create increment and decrement buttons
            increment_button = QPushButton(f'+ Joint {i+1}')
            increment_button.clicked.connect(lambda idx=i: self.adjust_joint_angle(idx, 0.1))
            decrement_button = QPushButton(f'- Joint {i+1}')
            decrement_button.clicked.connect(lambda idx=i: self.adjust_joint_angle(idx, -0.1))
            
            # Add widgets to layout
            button_layout = QHBoxLayout()
            button_layout.addWidget(increment_button)
            button_layout.addWidget(decrement_button)
            
            layout.addWidget(slider_label)
            layout.addWidget(slider)
            layout.addLayout(button_layout)
            
            # Store widgets
            self.labels.append(slider_label)
            self.sliders.append(slider)
            self.increment_buttons.append(increment_button)
            self.decrement_buttons.append(decrement_button)

        self.setLayout(layout)

    def update_joint_angle(self, value, idx):
        # Update the joint angle in radians
        angle = (value - 90) * math.pi / 180  # Convert from degrees to radians
        self.publisher.set_joint_angles([angle if i == idx else ang for i, ang in enumerate(self.publisher.joint_angles)])
        self.labels[idx].setText(f'Joint {idx+1} Angle (radians): {angle:.2f}')

    def adjust_joint_angle(self, idx, delta):
        # Adjust the joint angle by delta radians
        new_angle = self.publisher.joint_angles[idx] + delta
        # Ensure the angle stays within [0, 2*pi] radians
        new_angle = max(0.0, min(new_angle, 2 * math.pi))
        self.publisher.set_joint_angles([new_angle if i == idx else ang for i, ang in enumerate(self.publisher.joint_angles)])
        # Update slider and label
        slider_value = new_angle * 180 / math.pi + 90  # Convert from radians to degrees
        self.sliders[idx].setValue(slider_value)
        self.labels[idx].setText(f'Joint {idx+1} Angle (radians): {new_angle:.2f}')
        # Publish the updated angles
        self.publisher.publish_joint_angles()

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def closeEvent(self, event):
        # Clean up ROS 2 resources
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = JointControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
