import pybullet as p
import pyfirmata
import time
# go to pyfirmata.py in C:\Users\heman\AppData\Roaming\Python\Python312\site-packages\pyfirmata\pyfirmata.py
# change line number 185 from inspect.getargspec to inspect.getfullargspec
physicsclient = p.connect(p.GUI)

arm = p.loadURDF("./arm/edo_sim.urdf", useFixedBase=True)

sliders = []
for i in range(2):
    slider = p.addUserDebugParameter(f"joint_{i+1}", -3.14, 3.14, 0)
    sliders.append(slider)

port = 'COM10'
board = pyfirmata.Arduino(port)
servo_pin = board.get_pin('d:9:s')

def move_servo(angle):
    mapped_angle = int((angle + 180) * (255.0 / 360.0))
    servo_pin.write(mapped_angle)

move_servo(0) # to keep in default position of 0 angle

print("Starting simulation and servo control")
previous_position = 0
while True:
    target_position = p.readUserDebugParameter(sliders[0])
    
    if target_position != previous_position:
        p.setJointMotorControl2(arm, 0, p.POSITION_CONTROL, targetPosition=target_position)
        
        servo_angle = target_position * (180.0 / 3.14)
        move_servo(servo_angle)
        
        previous_position = target_position
    
    target_position_joint_2 = p.readUserDebugParameter(sliders[1])
    p.setJointMotorControl2(arm, 1, p.POSITION_CONTROL, targetPosition=target_position_joint_2)
    
    p.stepSimulation()
    time.sleep(1./240.)

print("Simulation and servo control complete")
board.exit()