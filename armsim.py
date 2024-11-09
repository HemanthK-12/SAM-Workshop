import pybullet as p
import pyfirmata
import time

# go to pyfirmata.py in C:\Users\heman\AppData\Roaming\Python\Python312\site-packages\pyfirmata\pyfirmata.py
# change line number 185 from inspect.getargspec to inspect.getfullargspec

physicsclient = p.connect(p.GUI) #connects to the physics engine and loads a GUI

arm = p.loadURDF("./r_arm/urdf/robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)
#spawns the urdf of arm at the base position of 0,0,0 and fixes the base

#joint_0 - base servo
#joint_1 - right servo
#joint_2 - left servo
#joint_3 - gripper servo
target_joint_names = ["joint_0", "joint_1", "joint_2", "joint_3"]

# Initialize PyFirmata
port = 'COM10'  # Update with your Arduino port from the arduino ide
board = pyfirmata.Arduino(port)
servo_pins = [9, 10, 11, 12]  # Update with your servo pins
servos = [board.get_pin(f'd:{pin}:s') for pin in servo_pins]

def move_servo(servo, angle):
    mapped_angle = int((angle + 3.14) * (180.0 / 6.28))  # Map from radians to degrees
    servo.write(mapped_angle) # Write the angle to the servo, serves the same purpose as digitalWrite

while True:
    p.stepSimulation()
    time.sleep(1./240.)

    keys = p.getKeyboardEvents()
    if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED: #when the key 'x' is pressed, it gives out the position of the end effector, which gives the joint positions through inverse kinematics
        # Get the end effector position(the end of the arm which grips)
        end_effector_state = p.getLinkState(arm, p.getNumJoints(arm) - 1)
        end_effector_pos = end_effector_state[4]  # Position is the 5th element in the state tuple
        end_effector_orn = end_effector_state[5]  # Orientation is the 6th element in the state tuple
        
        #can print this for finding out the position
        # print("End effector position:", end_effector_pos)
        # print("End effector orientation:", end_effector_orn)

        # Calculate joint positions through inverse kinematics to get the joint positions
        joint_positions = p.calculateInverseKinematics(arm, p.getNumJoints(arm) - 1, end_effector_pos, end_effector_orn)        
        
        # Move the servos based on the joint angles calculated now through inverse kinematics
        for index, joint_name in enumerate(target_joint_names):
            move_servo(servos[index], joint_positions[joint_name])
        
        break
board.exit()
p.disconnect()




#joint_0 - base servo
#joint_1 - right servo
#joint_2 - left servo
#joint_3 - gripper servo