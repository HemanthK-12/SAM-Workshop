import pybullet as p
import pyfirmata
import time
physicsclient = p.connect(p.GUI)

port = 'COM10'
board = pyfirmata.Arduino(port)
servo_pin = board.get_pin('d:9:s')
arm = p.loadURDF("./arm/edo_sim.urdf", useFixedBase=True)
def move_servo(angle):
    mapped_angle = int((angle + 180) * (255.0 / 360.0))
    servo_pin.write(mapped_angle)

move_servo(0) # to keep in default position of 0 angle

print("Starting simulation and servo control")
previous_position = 0

# Define the target endpoint positions
target_position_1 = [0.5, 0.5, 0.5]  # Example position 1 (x, y, z)
target_position_2 = [15, 15, 0.5]  # Example position 2 (x, y, z)

# Initialize the current target position
current_target_position = target_position_1

while True:
    # Calculate inverse kinematics for the current target endpoint position
    joint_positions = p.calculateInverseKinematics(arm, endEffectorLinkIndex=3, targetPosition=current_target_position)
    
    # Set the joint positions
    for i in range(2):
        p.setJointMotorControl2(arm, i, p.POSITION_CONTROL, targetPosition=joint_positions[i])
    
    # Move the servo based on the first joint position
    servo_angle = joint_positions[0] * (180.0 / 3.14)
    move_servo(servo_angle)
    
    p.stepSimulation()
    time.sleep(1./240.)
    
    # Switch target position after some time
    if current_target_position == target_position_1:
        current_target_position = target_position_2
    else:
        current_target_position = target_position_1
    
    # Add a delay to observe the movement
    time.sleep(0.5)

print("Simulation and servo control complete")
board.exit()