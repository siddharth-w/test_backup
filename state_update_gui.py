import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PyQt5.QtCore import Qt
import sys

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
        self.setWindowTitle('Joint Control GUI')

        # Create layout
        layout = QVBoxLayout()

        # Create sliders for each joint angle
        self.sliders = []
        self.labels = []
        for i in range(9):
            slider_label = QLabel(f'Joint {i+1} Angle (radians): 0.0')
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.setTickInterval(1)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(lambda value, idx=i: self.update_joint_angle(value, idx))
            self.labels.append(slider_label)
            self.sliders.append(slider)
            layout.addWidget(slider_label)
            layout.addWidget(slider)

        # Create a button to publish joint angles
        self.publish_button = QPushButton('Publish Joint Angles')
        self.publish_button.clicked.connect(self.publish_joint_angles)
        layout.addWidget(self.publish_button)

        self.setLayout(layout)

    def update_joint_angle(self, value, idx):
        # Update the joint angle in radians
        angle = (value - 90) * 3.14159 / 180  # Convert from degrees to radians
        self.publisher.set_joint_angles([angle if i == idx else ang for i, ang in enumerate(self.publisher.joint_angles)])
        self.labels[idx].setText(f'Joint {idx+1} Angle (radians): {angle:.2f}')

    def publish_joint_angles(self):
        # Publish the joint angles
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
