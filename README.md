# mpu6050_jetson

# MPU6050 for Jetson Nano

A Python library for interfacing MPU6050 6-axis accelerometer/gyroscope with NVIDIA Jetson Nano.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Features
- Simple Python interface for MPU6050
- Accelerometer, gyroscope, and temperature readings
- Built-in calibration routines
- Complementary filter for orientation (roll/pitch)
- Example scripts for common use cases
- No external dependencies beyond standard libraries

## Hardware Requirements
- NVIDIA Jetson Nano (any version)
- MPU6050 sensor module
- Jumper wires
- Breadboard (optional)

## Wiring Guide
| MPU6050 Pin | Jetson Nano Pin |
|-------------|-----------------|
| VCC | Pin 1 (3.3V) |
| GND | Pin 6 (GND) |
| SCL | Pin 5 (GPIO3, I2C SCL) |
| SDA | Pin 3 (GPIO2, I2C SDA) |
| AD0 | GND (for address 0x68) |

## Installation

### 1. Enable I2C on Jetson Nano
```bash
sudo apt update
sudo apt install -y python3-pip python3-smbus i2c-tools
sudo usermod -a -G i2c $USER
# Reboot or log out and back in
