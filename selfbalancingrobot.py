from selfbalancingrobot_mpu import MPU as SBRMPU
from machine import Pin, PWM

import math
import time


led = Pin("LED", Pin.OUT)
led.off()

servo = PWM(Pin(2))
servo.freq(50)
# factor = (8400 - 1600) / 180
factor = (8400 - 1600) / 180
# offset = 1600 + (8400 - 1600) / 2
offset = 1600 + 2800
servo_value = None

imu = SBRMPU()

period = 0.02

while True:
    comp_roll, comp_pitch = imu.get_roll_pitch()

    print("dt", imu.dt, "c_roll", comp_roll, "c_pitch", comp_pitch)

    if abs(comp_roll) < 5 and abs(comp_pitch) < 5:
        led.toggle()
    else:
        led.value(0)

    if servo_value is None:
        servo_value = max(min(int(- comp_roll * factor + offset), 8400), 1600)
    else:
        servo_value = int(0.0 * servo_value + 1.0 *
                          max(min(int(- comp_roll * factor + offset), 8400), 1600))
    servo.duty_u16(servo_value)

    time.sleep(period)
