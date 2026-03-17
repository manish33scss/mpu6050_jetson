#!/usr/bin/env python3
"""
MPU6050 Orientation Visualizer - HUD Style with Camera Calibration
Shows roll and pitch like an aircraft attitude indicator
- Applies camera calibration from JSON
- Loads ROI selection from JSON
- Undistorts and crops footage
- Overlays HUD on corrected image
"""

import cv2
import numpy as np
import time
import math
import sys
import os
import json
from mpu6050 import MPU6050

class OrientationVisualizer:
    def __init__(self):
        # ============================================
        # Load Camera Calibration
        # ============================================
        self.load_camera_calibration()
        
        # ============================================
        # Load ROI Selection
        # ============================================
        self.load_roi_selection()
        
        # ============================================
        # Initialize MPU6050
        # ============================================
        print("\n" + "="*60)
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
    
    def load_camera_calibration(self):
        """Load camera calibration from JSON file"""
        calib_file = 'calibration.json'
        
        if not os.path.exists(calib_file):
            print(f"ERROR: {calib_file} not found!")
            print("Please run camera calibration first.")
            sys.exit(1)
        
        with open(calib_file, 'r') as f:
            calib = json.load(f)
        
        # Extract camera matrix
        self.camera_matrix = np.array(calib['camera_matrix'])
        
        # Extract distortion coefficients
        self.dist_coeffs = np.array(calib['distortion_coefficients'])
        
        # Extract optimal camera matrix (for undistortion)
        if 'optimal_camera_matrix' in calib:
            self.optimal_matrix = np.array(calib['optimal_camera_matrix'])
        else:
            self.optimal_matrix = self.camera_matrix
        
        # Extract ROI from calibration (if available)
        if 'roi' in calib:
            self.calib_roi = calib['roi']
            print(f"Calibration ROI: {self.calib_roi}")
        else:
            self.calib_roi = None
        
        print(f" Loaded camera calibration:")
        print(f"   Camera matrix:\n{self.camera_matrix}")
        print(f"   Distortion: {self.dist_coeffs.flatten()}")
    
    def load_roi_selection(self):
        """Load ROI selection from JSON file"""
        roi_file = 'roi_settings.json'
        
        if not os.path.exists(roi_file):
            print(f" {roi_file} not found. Using full frame.")
            self.roi = None
            return
        
        with open(roi_file, 'r') as f:
            roi_data = json.load(f)
        
        # Get corners
        corners = roi_data['corners']
        
        # Calculate bounding rectangle from corners
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        
        self.roi = {
            'x': min(xs),
            'y': min(ys),
            'w': max(xs) - min(xs),
            'h': max(ys) - min(ys)
        }
        
        #print(f" Loaded ROI: {self.roi}")
    
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
        
        print("\n Initializing camera with GStreamer...")
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("Failed to open camera")
            sys.exit(1)
        
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        print("Camera initialized successfully")
    
    def process_frame(self, frame):
        """
        Apply camera calibration and ROI to frame
        Returns undistorted and cropped frame
        """
        h, w = frame.shape[:2]
        
        # Step 1: Undistort using camera calibration
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, 
            self.dist_coeffs, 
            (w, h), 
            1,  # alpha=1 retains all pixels
            (w, h)
        )
        
        # Undistort
        undistorted = cv2.undistort(
            frame, 
            self.camera_matrix, 
            self.dist_coeffs, 
            None, 
            new_camera_matrix
        )
        
        # Step 2: Apply calibration ROI (removes black edges)
        if roi is not None:
            x, y, roi_w, roi_h = roi
            if roi_w > 0 and roi_h > 0:
                undistorted = undistorted[y:y+roi_h, x:x+roi_w]
                #print(f"After calibration ROI: {undistorted.shape}")  # Debug
        
        # Step 3: Apply user-defined ROI from roi_settings.json (SCALED)
        if hasattr(self, 'roi') and self.roi is not None:
            # Calculate scaling factors
            scale_x = undistorted.shape[1] / w  # new width / original width
            scale_y = undistorted.shape[0] / h  # new height / original height
            
            # Scale ROI to new image size
            scaled_roi = {
                'x': int(self.roi['x'] * scale_x),
                'y': int(self.roi['y'] * scale_y),
                'w': int(self.roi['w'] * scale_x),
                'h': int(self.roi['h'] * scale_y)
            }
            
            # Ensure ROI is within bounds
            img_h, img_w = undistorted.shape[:2]
            
            if (scaled_roi['x'] >= 0 and scaled_roi['y'] >= 0 and 
                scaled_roi['x'] + scaled_roi['w'] <= img_w and 
                scaled_roi['y'] + scaled_roi['h'] <= img_h):
                
                undistorted = undistorted[
                    scaled_roi['y']:scaled_roi['y']+scaled_roi['h'],
                    scaled_roi['x']:scaled_roi['x']+scaled_roi['w']
                ]
                print(f"After user ROI: {undistorted.shape}")  # Debug
            else:
                print(f" Scaled ROI {scaled_roi} out of bounds for frame size {undistorted.shape[:2]}")
                print(f"   Using full undistorted frame")
        
        return undistorted
        
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
    
    def draw_hud(self, frame):
        """
        Super Simple HUD:
        - Yellow line = roll (tilts)
        - White cross (+) = pitch position (capped between 120-380)
        """
        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2

        # Roll line
        roll_rad = math.radians(self.roll)
        L = min(300, w//2)  # Adjust line length based on frame width
        
        dx = int(L * math.cos(roll_rad))
        dy = int(L * math.sin(roll_rad))
        
        # Horizon line (YELLOW) - fixed at center vertically
        x1 = cx - dx
        y1 = cy - dy
        x2 = cx + dx
        y2 = cy + dy
        
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
        
        # Pitch cross (+) - moves up/down with capping
        pitch_scale = 5  # pixels per degree
        pitch_offset = int(self.pitch * pitch_scale)
        
        # Calculate cross position (center - offset because screen Y increases downward)
        cross_x = cx
        cross_y = cy - pitch_offset
        
        # CAP the cross position to stay within frame
        min_y = int(cy * 0.3)
        max_y = int(cy * 1.7)
        cross_y = max(min_y, min(max_y, cross_y))
        
        # Draw cross (+)
        cross_size = min(20, h//20)
        cv2.line(frame, (cross_x - cross_size, cross_y), (cross_x + cross_size, cross_y), (255, 255, 255), 2)
        cv2.line(frame, (cross_x, cross_y - cross_size), (cross_x, cross_y + cross_size), (255, 255, 255), 2)
        
        # Draw center reference dot (small)
        cv2.circle(frame, (cx, cy), 3, (100, 100, 100), -1)
        
        return frame
    
    def draw_info_panel(self, frame):
        """Draw sensor data in top-left corner"""
        # Get raw sensor data
        accel = self.imu.get_accel()
        gyro = self.imu.get_gyro()
        temp = self.imu.get_temp()
        
        x, y = 15, 30
        line_height = 22
        
        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (280, 230), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        
        # Roll and Pitch
        cv2.putText(frame, f"Roll:  {self.roll:+7.2f} deg", (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"Pitch: {self.pitch:+7.2f} deg", (x, y + line_height), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Accelerometer
        cv2.putText(frame, f"AX: {accel[0]:+5.2f}g", (x, y + line_height*3), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"AY: {accel[1]:+5.2f}g", (x + 100, y + line_height*3), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"AZ: {accel[2]:+5.2f}g", (x + 200, y + line_height*3), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Gyroscope
        cv2.putText(frame, f"GX: {gyro[0]:+6.1f}", (x, y + line_height*5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"GY: {gyro[1]:+6.1f}", (x + 100, y + line_height*5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"GZ: {gyro[2]:+6.1f}", (x + 200, y + line_height*5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Temperature
        cv2.putText(frame, f"Temp: {temp:5.1f} C", (x, y + line_height*7), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    def run(self):
        """Main loop"""
        print("\n" + "="*60)
        print("MPU6050 HUD VISUALIZER with Camera Calibration")
        print("="*60)
        #print("Like an aircraft attitude indicator:")
        print("  • Yellow line = horizon (roll)")
        print("  • White cross = pitch position")
        print("  • Undistorted and cropped footage")
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
                
                # Apply camera calibration and ROI
                frame = self.process_frame(frame)
                
                # Update orientation
                self.update_orientation()
                
                # Draw HUD overlay
                frame = self.draw_hud(frame)
                
                # Draw info panel
                self.draw_info_panel(frame)
                
                # Calculate and display FPS
                fps_counter += 1
                if time.time() - fps_time >= 1.0:
                    fps = fps_counter
                    fps_counter = 0
                    fps_time = time.time()
                
                cv2.putText(frame, f"{fps} fps", (frame.shape[1]-80, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show frame
                cv2.imshow('MPU6050 HUD - Calibrated View', frame)
                
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
        print("Please ensure mpu6050.py is in the same directory")
        sys.exit(1)
    
    # Run visualizer
    viz = OrientationVisualizer()
    viz.run()
