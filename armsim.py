import pybullet as p
import pybullet_data
import time
# go to pyfirmata.py in C:\Users\heman\AppData\Roaming\Python\Python312\site-packages\pyfirmata\pyfirmata.py
# change line number 185 from inspect.getargspec to inspect.getfullargspec

#joint_0 - base servo
#joint_1 - right servo
#joint_2 - left servo
#joint_3 - gripper servo

# Connect to PyBullet
physicsclient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the URDF file
arm = p.loadURDF("./r_arm/urdf/robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Define the joint names to filter
target_joint_names = ["joint_0", "joint_1", "joint_2", "joint_3"]

print("Press 'x' to get the current joint angles and exit.")
while True:
    p.stepSimulation()
    time.sleep(1./240.)

    keys = p.getKeyboardEvents()
    if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED:
        # Get the number of joints
        num_joints = p.getNumJoints(arm)
        
        joint_angles = {}
        for i in range(num_joints):
            joint_info = p.getJointInfo(arm, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name in target_joint_names:
                joint_state = p.getJointState(arm, i)
                joint_angles[joint_name] = joint_state[0]  # Joint position is the first element in the state tuple
        
        print("Current joint angles:", joint_angles)
        break

print("Simulation ended.")
p.disconnect()