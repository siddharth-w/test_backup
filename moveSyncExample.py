from owl_client import OwlClient, Pose 
import time

_robot_ip = "10.42.0.3"
client = OwlClient(_robot_ip)

# Wait for Robot to be available to operate
while not client.is_running():
    time.sleep(0.2)

_toolspeed = 100  # mm/sec

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
client.move_to_pose(_pose1, _toolspeed)

print("Moving to Pose2")
# Pose 2
_pose2 = Pose()
_pose2.x = -0.176
_pose2.y = -0.240204
_pose2.z = 0.159684
_pose2.roll = 3.1376
_pose2.pitch = 1
_pose2.yaw = 1.56449
# Move to Pose2
client.move_to_pose(_pose2, _toolspeed)

# print("Moving to Pose3")
# # Pose 3
# _pose3 = Pose()
# _pose3.x = -0.176
# _pose3.y = -0.240204
# _pose3.z = 0.489203
# _pose3.roll = 3.1376
# _pose3.pitch = -0.087288
# _pose3.yaw = 1.56449
# # Move to Pose3
# client.move_to_pose(_pose3, _toolspeed)


# print("Moving to Pose4")
# # Pose 4
# _pose4 = Pose()
# _pose4.x = -0.211
# _pose4.y = -0.169035
# _pose4.z = 0.489204
# _pose4.roll = 3.1376
# _pose4.pitch = -0.087288
# _pose4.yaw = 1.56449
# # Move to Pose4
# client.move_to_pose(_pose4, _toolspeed)


# print("Moving to Pose5")
# # Pose 5
# _pose5 = Pose()
# _pose5.x = -0.211
# _pose5.y = -0.169035
# _pose5.z = 0.159684
# _pose5.roll = 3.1376
# _pose5.pitch = -0.087288
# _pose5.yaw = 1.56449
# # Move to Pose5
# client.move_to_pose(_pose5, _toolspeed)
