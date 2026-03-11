#!/usr/bin/env python3
"""
MPU6050 Driver for Jetson Nano
Simple and easy to use
"""

import smbus
import time
import math

class MPU6050:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus.SMBus(bus)
        self.address = address
        
        # Wake up the sensor
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        time.sleep(0.1)
        
        # Configure accelerometer (±2g)
        self.bus.write_byte_data(self.address, 0x1C, 0x00)
        
        # Configure gyroscope (±250°/s)
        self.bus.write_byte_data(self.address, 0x1B, 0x00)
        
        # Verify connection
        who_am_i = self.bus.read_byte_data(self.address, 0x75)
        print(f"MPU6050 detected (ID: 0x{who_am_i:02x})")
        
        self.gyro_bias = [0, 0, 0]
        
    def read_raw(self, reg):
        """Read 16-bit value"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg+1)
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val
    
    def get_accel(self):
        """Get accelerometer data (g)"""
        x = self.read_raw(0x3B) / 16384.0
        y = self.read_raw(0x3D) / 16384.0
        z = self.read_raw(0x3F) / 16384.0
        return (x, y, z)
    
    def get_gyro(self):
        """Get gyroscope data (degrees/sec)"""
        x = self.read_raw(0x43) / 131.0
        y = self.read_raw(0x45) / 131.0
        z = self.read_raw(0x47) / 131.0
        
        # Apply bias if calibrated
        if any(self.gyro_bias):
            x -= self.gyro_bias[0]
            y -= self.gyro_bias[1]
            z -= self.gyro_bias[2]
        
        return (x, y, z)
    
    def get_temp(self):
        """Get temperature (Celsius)"""
        temp = self.read_raw(0x41) / 340.0 + 36.53
        return temp
    
    def calibrate_gyro(self, samples=100):
        """Calibrate gyroscope (keep sensor still)"""
        print(f"Calibrating ({samples} samples)... keep sensor still")
        gx = gy = gz = 0
        
        for i in range(samples):
            gx += self.read_raw(0x43)
            gy += self.read_raw(0x45)
            gz += self.read_raw(0x47)
            time.sleep(0.01)
        
        self.gyro_bias = [
            (gx / samples) / 131.0,
            (gy / samples) / 131.0,
            (gz / samples) / 131.0
        ]
        
        print(f"Gyro bias: X={self.gyro_bias[0]:+.2f}, Y={self.gyro_bias[1]:+.2f}, Z={self.gyro_bias[2]:+.2f}")
        return self.gyro_bias
    
    def get_all(self):
        """Get all sensor data"""
        return self.get_accel(), self.get_gyro(), self.get_temp()

# Simple test if run directly
if __name__ == "__main__":
    print("Testing MPU6050...")
    imu = MPU6050()
    
    try:
        while True:
            accel, gyro, temp = imu.get_all()
            print(f"\rAccel(g): X={accel[0]:6.3f} Y={accel[1]:6.3f} Z={accel[2]:6.3f} | "
                  f"Gyro(dps): X={gyro[0]:8.2f} Y={gyro[1]:8.2f} Z={gyro[2]:8.2f} | "
                  f"Temp: {temp:6.2f}°C", end='')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nDone")
