from imu import MPU6050
from machine import Pin, I2C

import math
import time


class MPU():
    
    def __init__(self, sda=0, scl=1, parent=None):
        
        i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=400000)

        self.imu = MPU6050(i2c)
        self.imu.accel_range = 0
        self.imu.gyro_range = 1
        # self.imu.filter_range = 0

        # imu calib errors
        self.aXerr = 0.03252075
        self.aYerr = 0.001260986
        self.aZerr = -0.03212547
        self.gXerr = -1.14307
        self.gYerr = -0.0766717
        self.gZerr = 0.7535422

        self.dt = 0
        self.pc = 99.0  # 99.99
        self.comp_roll = None
        self.comp_pitch = None
        self.gyro_roll = None
        self.gyro_pitch = None
        self.gyro_yaw = 0

        print("MPU6050 Initialised successfully.")
        # print(self.imu.accel_range, self.imu.gyro_range)

        self.start = time.ticks_us()
        
        
    def get_roll_pitch(self, in_loop=True):
        
        now = time.ticks_us()
        self.dt = time.ticks_diff(now, self.start) / 1000000
        self.start = now
        
        # print(imu.accel.xyz, imu.gyro.xyz, imu.temperature, end='\r')
        
        # Read sensor data
        aX = self.imu.accel.x - self.aXerr
        aY = self.imu.accel.y - self.aYerr
        aZ = self.imu.accel.z - self.aZerr
        gX = self.imu.gyro.x - self.gXerr
        gY = self.imu.gyro.y - self.gYerr
        gZ = self.imu.gyro.z - self.gZerr
        #tem = self.imu.temperature
        #print(aX, "\t", aY, "\t", aZ, "\t", gX, "\t", gY, "\t", gZ, "\t", tem, "        ", end="\r")
        #print(ax, "\t", ay, "\t", az)
        #print(gX, "\t", gY, "\t", gZ)
        
        # Calculate gyro rate [deg/sec]
        gyroXRate = gX  # / gyro_precision
        gyroYRate = gY  # / gyro_precision
        gyroZRate = gZ  # / gyro_precision
        
        # Calculate gyro angle [deg]
        gyroXAngle = gyroXRate * self.dt
        gyroYAngle = gyroYRate * self.dt
        gyroZAngle = gyroZRate * self.dt
        
        # Calculate roll and pitch
        roll = math.atan2(aY , math.sqrt((aX**2) + (aZ**2))) * (180/math.pi)
        pitch = math.atan2(- aX , math.sqrt((aY**2) + (aZ**2))) * (180/math.pi)

        roll -= pitch * math.sin(gyroZAngle)
        pitch += roll * math.sin(gyroZAngle)
        
        """
        if self.gyro_roll is None or not in_loop:
            self.gyro_roll = roll
        else:
            self.gyro_roll += gyroXAngle
        
        if self.gyro_pitch is None or not in_loop:
            self.gyro_pitch = pitch
        else:
            self.gyro_pitch += gyroYAngle
            
        self.gyro_yaw += gyroZAngle
        """
        
        # Calculate composite angle
        if self.comp_roll is None or not in_loop:
            self.comp_roll = roll
        else:
            self.comp_roll = (self.pc / 100) * (self.comp_roll + gyroXAngle) + (1 - (self.pc / 100)) * roll
        
        if self.comp_pitch is None or not in_loop:
            self.comp_pitch = pitch
        else:
            self.comp_pitch = (self.pc / 100) * (self.comp_pitch + gyroYAngle) + (1 - (self.pc / 100)) * pitch
        
        #print("roll", roll, "pitch", pitch)
        ##print("roll", roll, "pitch", pitch, "c_roll", self.comp_roll, "c_pitch", self.comp_pitch)
        #print("dt", dt, "roll", roll, "g_roll", gyro_roll, "c_roll", comp_roll)
        #print("pitch", pitch, "c_pitch", comp_pitch)
        #print(gyroXAngle, gyroYAngle, gyroZAngle)
        #print(gyro_yaw)
        
        return self.comp_roll, self.comp_pitch

