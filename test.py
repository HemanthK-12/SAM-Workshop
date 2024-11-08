# go to pyfirmata.py in C:\Users\heman\AppData\Roaming\Python\Python312\site-packages\pyfirmata\pyfirmata.py
# change line number 185 from inspect.getargspec to inspect.getfullargspec
import pyfirmata
import time

port = 'COM10'

board = pyfirmata.Arduino(port)

servo_pin = board.get_pin('d:9:s')  # 'd' for digital, '9' for pin number, 's' for servo

def move_servo(angle):
    servo_pin.write(angle)
    time.sleep(0.5)  # Wait for the servo to reach the position

print("Starting servo control")
while True:
    move_servo(0)
    move_servo(90)
    move_servo(180)
    move_servo(90)
    move_servo(0)
print("Servo control complete")
board.exit()