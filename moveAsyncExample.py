from owl_client import OwlClient, Pose
import time

_robot_ip = "10.42.0.3"
client = OwlClient(_robot_ip)
_toolspeed = 100  # mm/sec
print("Moving to Pose")
# Pose
_pose1 = Pose()
_pose1.x = -0.176
_pose1.y = -0.240204
_pose1.z = 0.489203
_pose1.roll = 3.1376
_pose1.pitch = -0.087288
_pose1.yaw = 1.56449

# Move to Pose1 asynchronously
client.move_to_pose(_pose1, _toolspeed, wait=False)
time.sleep(5)
client.move_abort()
