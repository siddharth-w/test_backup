from owl_client import OwlClient, Joint
import time

client = OwlClient("10.42.0.3")
jointSpeed = 1 #degrees/sec

# Wait for robot to be available
while not client.is_running():
    time.sleep(0.2)

#create joint goals for robot

#zero configuration
zero_position = Joint()
zero_position.Base = 0.0
zero_position.Shoulder = 0.0
zero_position.Elbow  = 0.0
zero_position.Wrist1 = 0.0
zero_position.Wrist2 = 0.0
zero_position.Wrist3 = 0.0

#valid configuration
valid_position = Joint()
valid_position.Base = 0.2
valid_position.Shoulder = 0.0
valid_position.Elbow  = 0.0  #(-90)
valid_position.Wrist1 = 0.0
valid_position.Wrist2 = 0.0
valid_position.Wrist3 = 0.0

count = 0
while count < 1 :
    client.move_to_joint(zero_position, jointSpeed)
    time.sleep(5)
    client.move_to_joint(valid_position, jointSpeed)
    time.sleep(1)
    count += 1
