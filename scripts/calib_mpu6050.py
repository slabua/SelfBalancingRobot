from imu import MPU6050
from machine import Pin, I2C
from time import sleep

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

imu = MPU6050(i2c)
imu.accel_range = 0
imu.gyro_range = 0

period = 0.02
n_samples = 1000

print(imu.accel_range, imu.gyro_range)

aX = 0
aY = 0
aZ = 0
gX = 0
gY = 0
gZ = 0

for n in range(n_samples):
    #print(imu.accel.xyz,imu.gyro.xyz,imu.temperature,end='\r')
    
    # Read sensor data
    aX += imu.accel.x
    aY += imu.accel.y
    aZ += imu.accel.z
    gX += imu.gyro.x
    gY += imu.gyro.y
    gZ += imu.gyro.z
    
    sleep(period)

aX /= n_samples
aY /= n_samples
aZ /= n_samples
gX /= n_samples
gY /= n_samples
gZ /= n_samples

aZ -= 1

print(aX, "\t", aY, "\t", aZ, "\t", gX, "\t", gY, "\t", gZ, "        ", end="\r")
