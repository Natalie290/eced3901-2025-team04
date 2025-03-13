from machine import Pin, PWM
from time import sleep

# Define servo pins
servo1 = PWM(Pin(17))
servo2 = PWM(Pin(18))

# Set PWM frequency to 50 Hz (standard for servos)
servo1.freq(50)
servo2.freq(50)

def set_angle(servo, angle):
    # Convert angle (0 to 180) to duty cycle (1000 to 9000)
    duty = int((angle / 180) * 8000 + 1000)
    servo.duty_u16(duty)

def move_servo(servo, start, end, delay_time):
    step = 1 if start < end else -1
    for pos in range(start, end + step, step):
        set_angle(servo, pos)
        sleep(delay_time / 1000)

while True:
    # Move first servo from 0 to 180 degrees and back
    move_servo(servo1, 0, 180, 15)
    move_servo(servo1, 180, 0, 15)

    # Move second servo from 0 to 180 degrees and back
    move_servo(servo2, 0, 180, 15)
    move_servo(servo2, 180, 0, 15)
