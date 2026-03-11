#!/usr/bin/env python3
"""
Author : Meego
MPU6050 Orientation Visualizer - Because my mpu is mounted veritcally behind the camera, this code might not produce same results
Corrected for your specific mounting:
- X axis points DOWN (reads +1g when level)
- Y axis points LEFT (reads +0.9g when rolled right)
- Z axis points FORWARD
- Roll direction corrected
- This code is created by using copilot in more than one phases.
"""

import cv2
import numpy as np
import time
import math
import sys
import os
from mpu6050 import MPU6050

class OrientationVisualizer:
    def __init__(self):
        # Initialize MPU6050
        print("Initializing MPU6050...")
        self.imu = MPU6050()
        
        # Calibration instructions
        print("\n" + "="*60)
        print("CALIBRATION INSTRUCTIONS:")
        print("="*60)
        print("1. Place camera on flat table, pointing FORWARD")
        print("2. Camera should be LEVEL (not tilted)")
        print("3. This will be your ZERO position")
        print("="*60)
        input("Press Enter when camera is level and pointing straight ahead...")
        
        # Calibrate gyroscope
        print("\nCalibrating gyroscope... keep camera perfectly still!")
        self.imu.calibrate_gyro()
        
        # Orientation variables
        self.roll = 0.0   # Rotation around X axis (tilt left/right)
        self.pitch = 0.0  # Rotation around Y axis (tilt up/down)
        
        # Complementary filter parameters
        self.alpha = 0.96
        self.last_time = time.time()
        
        # Initialize camera
        self.init_camera()
        
    def init_camera(self):
        """Initialize Jetson Nano camera with GStreamer"""
        gst_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=2"
        )
        
        print("\nInitializing camera with GStreamer...")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("Failed to open camera")
            sys.exit(1)
        
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        print("Camera initialized successfully")
    
    def update_orientation(self):
        """
        Update roll and pitch using complementary filter
        Based on your test results:
        - X axis points DOWN (+1g when level)
        - Y axis points LEFT (+0.9g when rolled right)
        - Z axis points FORWARD
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Read sensor data
        accel = self.imu.get_accel()
        gyro = self.imu.get_gyro()
        
        ax, ay, az = accel
        gx, gy, gz = gyro
        
        # Calculate roll from accelerometer
        roll_acc = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
        
        # Calculate pitch from accelerometer
        pitch_acc = math.atan2(-az, math.sqrt(ax**2 + ay**2)) * 180 / math.pi
        
        # Integrate gyro data
        self.roll += gx * dt
        self.pitch += gz * dt  # Using gz for pitch rate
        
        # Complementary filter
        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc
        
        return self.roll, self.pitch
    
    def draw_pitch_axis(self, frame, pitch, cx, cy, L=70):
 
	    """
	    Draw axes where pitch rotates the X and Z axes around Y.
	    Correct physics:
	    - X axis (RED): rotates with pitch
	    - Y axis (GREEN): fixed (pitch axis)
	    - Z axis (BLUE): rotates with pitch
	    """

	    pitch_r = math.radians(pitch)

	    # Y-axis rotation matrix
	    Ry = np.array([
		[math.cos(pitch_r), 0, math.sin(pitch_r)],
		[0, 1, 0],
		[-math.sin(pitch_r), 0, math.cos(pitch_r)]
	    ])

	    # Base axes
	    x_base = np.array([L, 0, 0])   # X axis
	    y_base = np.array([0, L, 0])   # Y axis
	    z_base = np.array([0, 0, L])   # Z axis

	    # Rotate axes
	    x_rot = Ry @ x_base
	    y_rot = Ry @ y_base
	    z_rot = Ry @ z_base

	    # Project to screen coordinates
	    x_end = (cx + int(x_rot[0]), cy - int(x_rot[1]))
	    y_end = (cx + int(y_rot[0]), cy - int(y_rot[1]))
	    z_end = (cx + int(z_rot[0]), cy - int(z_rot[1]))

	    # Draw X axis (RED)
	    cv2.arrowedLine(frame, (cx, cy), x_end, (0, 0, 255), 2, tipLength=0.2)
	    cv2.putText(frame, "X", (x_end[0] + 5, x_end[1] - 5),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

	    # Draw Y axis (GREEN) - pitch axis (stays fixed)
	    cv2.arrowedLine(frame, (cx, cy), y_end, (0, 255, 0), 3, tipLength=0.2)
	    cv2.putText(frame, "Y", (y_end[0] + 5, y_end[1] - 5),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

	    # Draw Z axis (BLUE)
	    cv2.arrowedLine(frame, (cx, cy), z_end, (255, 0, 0), 2, tipLength=0.2)
	    cv2.putText(frame, "Z", (z_end[0] + 5, z_end[1] - 5),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

	    # Center marker
	    cv2.circle(frame, (cx, cy), 3, (255, 255, 255), -1)

	    # Pitch label
	    cv2.putText(frame, f"Pitch: {pitch:+.1f} deg",
		        (cx - 50, cy - 50),
		        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

	    return frame
    
    def draw_roll_line(self, frame):
        """Draw red roll line in center (corrected direction)"""
        h, w = frame.shape[:2]
        center_x = w // 2
        center_y = h // 2
        
        # Roll matches camera movement
        roll_rad = math.radians(self.roll)
        
        line_length = 180
        x1 = center_x - int(line_length * math.cos(roll_rad))
        y1 = center_y - int(line_length * math.sin(roll_rad))
        x2 = center_x + int(line_length * math.cos(roll_rad))
        y2 = center_y + int(line_length * math.sin(roll_rad))
        
        # Draw red roll line
        cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
        
        # Draw center point
        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)
        
        # Draw reference horizontal line (faint)
        cv2.line(frame, 
                (center_x - line_length, center_y),
                (center_x + line_length, center_y),
                (100, 100, 100), 1)
        
        # Add roll value
        cv2.putText(frame, f"Roll: {self.roll:+.1f} deg", (center_x - 60, center_y - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return frame
    
    def draw_info_panel(self, frame):
        """Draw all sensor data in top-left corner"""
        # Get raw sensor data
        accel = self.imu.get_accel()
        gyro = self.imu.get_gyro()
        temp = self.imu.get_temp()
        
        x, y = 15, 30
        line_height = 22
        
        # Title
        cv2.putText(frame, "SENSOR DATA", (x, y-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Roll and Pitch
        cv2.putText(frame, f"Roll:  {self.roll:+7.2f} deg", (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(frame, f"Pitch: {self.pitch:+7.2f} deg", (x, y + line_height), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Accelerometer (g)
        cv2.putText(frame, f"Accel X: {accel[0]:+6.3f} g", (x, y + line_height*3), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Accel Y: {accel[1]:+6.3f} g", (x, y + line_height*4), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Accel Z: {accel[2]:+6.3f} g", (x, y + line_height*5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Gyroscope (deg/s)
        cv2.putText(frame, f"Gyro X: {gyro[0]:+7.2f} deg/s", (x, y + line_height*7), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Gyro Y: {gyro[1]:+7.2f} deg/s", (x, y + line_height*8), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Gyro Z: {gyro[2]:+7.2f} deg/s", (x, y + line_height*9), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Temperature
        cv2.putText(frame, f"Temp: {temp:5.1f} C", (x, y + line_height*11), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # Mounting info
        cv2.putText(frame, "Mount: X down, Y left, Z forward", (x, y + line_height*13), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        cv2.putText(frame, "GREEN Y axis moves with pitch", (x, y + line_height*14), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(frame, "RED X and BLUE Z are fixed", (x, y + line_height*15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    
    def run(self):
        """Main loop"""
        print("\n" + "="*60)
        print("MPU6050 PITCH VISUALIZER")
        print("="*60)
        print("Mounting: X down, Y left, Z forward")
        print("\nExpectations when level on table:")
        print("  • Roll: 0°")
        print("  • Pitch: 0°")
        print("  • X axis: +1.0g")
        print("\nVisualization:")
        print("  • Center RED line = Roll")
        print("  • Corner GREEN arrow = Pitch (moves)")
        print("  • RED X and BLUE Z arrows = Fixed reference")
        print("\nControls:")
        print("  • 'q' - quit")
        print("  • 'c' - recalibrate gyroscope")
        print("  • 'r' - reset orientation")
        print("="*60)
        
        # FPS calculation
        fps_counter = 0
        fps = 0
        fps_time = time.time()
        
        try:
            while True:
                # Read camera frame
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                
                h, w = frame.shape[:2]
                
                # Update orientation
                self.update_orientation()
                
                # Draw visualizations
                frame = self.draw_roll_line(frame)                          # Center roll line
                frame = self.draw_pitch_axis(frame, self.pitch, w-120, h-80, L=60)  # Bottom-right axes
                self.draw_info_panel(frame)                                 # Top-left info
                
                # Calculate and display FPS
                fps_counter += 1
                if time.time() - fps_time >= 1.0:
                    fps = fps_counter
                    fps_counter = 0
                    fps_time = time.time()
                
                cv2.putText(frame, f"{fps} fps", (w-80, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show frame
                cv2.imshow('MPU6050 - Green Y moves with pitch', frame)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    print("\nRecalibrating gyroscope...")
                    print("Keep camera perfectly still!")
                    input("Press Enter when ready...")
                    self.imu.calibrate_gyro()
                    print("Calibration complete!")
                elif key == ord('r'):
                    print("\nResetting orientation...")
                    self.roll = 0.0
                    self.pitch = 0.0
                    print("Orientation reset")
                    
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\nCleaning up...")
        self.cap.release()
        cv2.destroyAllWindows()
        print("Done")

if __name__ == "__main__":
    # Check if mpu6050.py exists
    if not os.path.exists('mpu6050.py'):
        print("ERROR: mpu6050.py not found in current directory")
        print("Please download it from: https://github.com/yourusername/mpu6050-jetson")
        sys.exit(1)
    
    # Run visualizer
    viz = OrientationVisualizer()
    viz.run()
