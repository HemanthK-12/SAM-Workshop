import pyfirmata
import time

# Initialize PyFirmata
port = 'COM10'  # Update with your Arduino port
board = pyfirmata.Arduino(port)

# Define the servo pins
servo_pins = [3, 9, 10]
servos = [board.get_pin(f'd:{pin}:s') for pin in servo_pins]

# Function to move servo to a specific angle
def move_servo(servo, angle):
    servo.write(angle)  # Write the angle to the servo

# Test each servo by moving it to different positions

while True:
    for angle in range(0, 181, 30):  # Move from 0 to 180 degrees in steps of 30
        for servo in servos:
            move_servo(servo, angle)
        time.sleep(1)  # Wait for 1 second

    for angle in range(180, -1, -30):  # Move from 180 to 0 degrees in steps of 30
        for servo in servos:
            move_servo(servo, angle)
        time.sleep(1)  # Wait for 1 second