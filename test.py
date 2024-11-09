import pybullet as p
import pyfirmata
import time

# go to pyfirmata.py in C:\Users\heman\AppData\Roaming\Python\Python312\site-packages\pyfirmata\pyfirmata.py
# change line number 185 from inspect.getargspec to inspect.getfullargspec

physicsclient = p.connect(p.GUI) #connects to the physics engine and loads a GUI

arm = p.loadURDF("./r_arm/urdf/robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)
#spawns the urdf of arm at the base position of 0,0,0 and fixes the base

#joint_0 - base servo - pin3
#joint_1 - right servo - pin10
#joint_2 - left servo - pin9
target_joint_names = ["joint_0", "joint_1", "joint_2"]

board = pyfirmata.Arduino('COM10')
servo_pins = [3,10,9]  # Update with your servo pins
servos = [board.get_pin(f'd:{pin}:s') for pin in servo_pins]

def move_servo(servo, angle):
    mapped_angle = int((angle + 3.14) * (180.0 / 6.28))  # Map from radians to degrees
    servo.write(mapped_angle) # Write the angle to the servo, serves the same purpose as digitalWrite

while True:
    p.stepSimulation()
    time.sleep(1./240.)

    keys = p.getKeyboardEvents()
    if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED: 
        #get the positions of all the joints i.e joint_0, joint_1, joint_2, joint_3
        joint_positions = []
        for i in range(len(target_joint_names)):
            joint_state = p.getJointState(arm, i)
            joint_positions.append(joint_state[0])  # Joint position is the first element in the state tuple

        print("Current joint positions:", joint_positions)

        for i, joint_position in enumerate(joint_positions):
            move_servo(servos[i], joint_position)

