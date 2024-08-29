from owl_client import OwlClient, Pose 
import time

_robot_ip = "10.42.0.3"
client = OwlClient(_robot_ip)

# Wait for Robot to be available to operate
while not client.is_running():
    time.sleep(0.2)
    toolspeed = 100  # mm/sec
    print("Moving to Pose1")
    # Pose 1
    _pose1 = Pose()
    _pose1.x = -0.176
    _pose1.y = -0.240204
    _pose1.z = 0.489203
    _pose1.roll = 3.1376
    _pose1.pitch = -0.087288
    _pose1.yaw = 1.56449
    # Move to Pose1
    client.move_to_pose(_pose1, toolspeed)
    time.sleep(2)
    print("Moving to Pose2")
    # Pose 2
    _pose2 = Pose()
    _pose2.x = -0.176
    _pose2.y = -0.240204
    _pose2.z = 0.159684
    _pose2.roll = 3.1376
    _pose2.pitch = -0.087288
    _pose2.yaw = 1.56449
    # Move to Pose2
    client.move_to_pose(_pose2, toolspeed)

