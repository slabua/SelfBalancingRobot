# -*- coding: utf-8 -*-
"""Self Balancing Robot
"""

from machine import Pin, PWM
from PID import PID
from selfbalancingrobot_mpu import MPU as SBRMPU

import time

__author__ = "Salvatore La Bua"
__copyright__ = "Copyright 2022, Salvatore La Bua"
__license__ = "GPL"
__version__ = "0.1.0"
__maintainer__ = "Salvatore La Bua"
__email__ = "slabua@gmail.com"
__status__ = "Development"


# Parameters
PERIOD = 0.02

# LED
led = Pin("LED", Pin.OUT)
led.off()

# Servo
servo = PWM(Pin(2))
servo.freq(50)
# factor = (8400 - 1600) / 180
factor = (8400 - 1600) / 180
# offset = 1600 + (8400 - 1600) / 2
offset = 1600 + 2800
servo_value = None

# IMU
imu = SBRMPU()

# PID
pid = PID(1, 0.1, 1, setpoint=0, scale='us')
pid.output_limits = (1600, 8400)

while True:
    comp_roll, comp_pitch = imu.get_roll_pitch()

    print("dt", imu.dt, "c_roll", comp_roll, "c_pitch", comp_pitch)

    if abs(comp_roll) < 5 and abs(comp_pitch) < 5:
        led.toggle()
    else:
        led.value(0)

    # pid.setpoint = (comp_roll * factor + offset)
    # servo_value = int(pid(-comp_roll))

    if servo_value is None:
        servo_value = max(min(int(- comp_roll * factor + offset), 8400), 1600)
    else:
        servo_value = int(0.0 * servo_value + 1.0 *
                          max(min(int(- comp_roll * factor + offset), 8400), 1600))
    servo.duty_u16(servo_value)

    time.sleep(PERIOD)
