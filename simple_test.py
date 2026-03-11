#!/usr/bin/env python3
"""
Simple test for MPU6050 on Jetson Nano
"""

from mpu6050 import MPU6050
import time
import sys

def main():
    print("MPU6050 Test on Jetson Nano")
    print("-" * 40)
    
    # Initialize sensor
    try:
        imu = MPU6050()
    except Exception as e:
        print(f"Failed to initialize: {e}")
        print("\nTroubleshooting:")
        print("1. Check wiring: VCC->Pin1, GND->Pin6, SDA->Pin3, SCL->Pin5")
        print("2. Run: sudo i2cdetect -y -r 1 (should show 68)")
        sys.exit(1)
    
    # Optional: calibrate gyroscope
    cal = input("Calibrate gyroscope? (y/n): ").lower()
    if cal == 'y':
        input("Place sensor on flat surface and press Enter...")
        imu.calibrate_gyro()
    
    print("\nReading sensor data (Ctrl+C to stop)\n")
    
    try:
        while True:
            accel, gyro, temp = imu.get_all()
            print(f"\rAccel(g): X={accel[0]:+7.3f} Y={accel[1]:+7.3f} Z={accel[2]:+7.3f} | "
                  f"Gyro(dps): X={gyro[0]:+8.2f} Y={gyro[1]:+8.2f} Z={gyro[2]:+8.2f} | "
                  f"Temp: {temp:+6.2f}°C", end='')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nTest completed")

if __name__ == "__main__":
    main()
