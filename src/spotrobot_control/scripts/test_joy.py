#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import termios
import tty
import sys
import select
import os
import cv2
import mediapipe as mp
import numpy as np

# Key codes for special keys
ESC_KEY = '\x1b'

# Default Joy message values
DEFAULT_AXES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 8 axes
DEFAULT_BUTTONS = [0] * 11  # 11 buttons

MSG_TEMPLATE = """
Keyboard Teleop for Quadruped Robot
------------------------------------
Movement Controls (Hold Key):
  W: Forward         C : Crawl mode
  S: Stand mode      Q : setup after standup
  I : IMU            R : Stand UP
  D : Turn Right     A : Turn Left
  Z : Backward
"""

class KeyboardTeleopJoy(Node):
    def __init__(self):
        super().__init__("keyboard_teleop_joy_node")
        self.publisher_ = self.create_publisher(Joy, "/spot_joy/joy_ramped", 10)

        self.current_joy_msg = Joy()
        self.current_joy_msg.axes = list(DEFAULT_AXES)
        self.current_joy_msg.buttons = list(DEFAULT_BUTTONS)


        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.arm_connections = [
            (self.mp_pose.PoseLandmark.LEFT_SHOULDER, self.mp_pose.PoseLandmark.LEFT_ELBOW),
            (self.mp_pose.PoseLandmark.LEFT_ELBOW, self.mp_pose.PoseLandmark.LEFT_WRIST),
            (self.mp_pose.PoseLandmark.RIGHT_SHOULDER, self.mp_pose.PoseLandmark.RIGHT_ELBOW),
            (self.mp_pose.PoseLandmark.RIGHT_ELBOW, self.mp_pose.PoseLandmark.RIGHT_WRIST)
        ]
        # Custom drawing specs
        self.point_spec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 255, 0),  # Green points
            thickness=-1,  # Filled
            circle_radius=5  # Size
        )
        self.line_spec = mp.solutions.drawing_utils.DrawingSpec(
            color=(0, 255, 0),  # Green lines
            thickness=2
        )
        self.cap = cv2.VideoCapture(0)
        self.right_arm_up = False
        self.left_arm_up = False
        self.right_arm_bent = False
        self.left_arm_bent = False

        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.process_gestures)
        self.get_logger().info("Keyboard Teleop Joy Node Started.")
        self.step = 1
        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info(MSG_TEMPLATE)



    def is_arm_up(self, landmarks, arm_side):
        """Check if arm is raised (shoulder to wrist vertical movement)"""
        shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER if arm_side == "left" 
                           else self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST if arm_side == "left" 
                         else self.mp_pose.PoseLandmark.RIGHT_WRIST]
        
        # Arm is considered "up" if wrist is above shoulder
        return wrist.y < shoulder.y

    def is_arm_bent(self, landmarks, arm_side):
        """Check if arm is bent (elbow angle < 120 degrees)"""
        shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER if arm_side == "left" 
                           else self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        elbow = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW if arm_side == "left" 
                         else self.mp_pose.PoseLandmark.RIGHT_ELBOW]
        wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST if arm_side == "left" 
                         else self.mp_pose.PoseLandmark.RIGHT_WRIST]
        
         # Convert to numpy arrays
        shoulder = np.array([shoulder.x, shoulder.y])
        elbow = np.array([elbow.x, elbow.y])
        wrist = np.array([wrist.x, wrist.y])
        
        # Calculate vectors
        upper_arm = elbow - shoulder
        forearm = wrist - elbow
        
        # Calculate angle (0-180 degrees)
        angle = np.degrees(np.arctan2(
            np.linalg.norm(np.cross(upper_arm, forearm)),
            np.dot(upper_arm, forearm)
        ))
        return angle > 45

    def process_gestures(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Convert to RGB (MediaPipe requirement)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)
        
        if results.pose_landmarks:

            h, w, _ = frame.shape

            for connection in self.arm_connections:
                start_idx = connection[0].value
                end_idx = connection[1].value
                
                start_point = results.pose_landmarks.landmark[start_idx]
                end_point = results.pose_landmarks.landmark[end_idx]
                
                # Convert normalized coordinates to pixels
                start_px = (int(start_point.x * w), int(start_point.y * h))
                end_px = (int(end_point.x * w), int(end_point.y * h))
                
                # Draw line
                cv2.line(frame, start_px, end_px, 
                        self.line_spec.color, self.line_spec.thickness)
                
                # Draw points (will automatically deduplicate)
                cv2.circle(frame, start_px, self.point_spec.circle_radius,
                        self.point_spec.color, self.point_spec.thickness)
                cv2.circle(frame, end_px, self.point_spec.circle_radius,
                        self.point_spec.color, self.point_spec.thickness)
                # Draw landmarks (optional)

            # self.mp_draw.draw_landmarks(
            #     frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            
            # Check arm states
            self.right_arm_up = self.is_arm_up(results.pose_landmarks.landmark, "right")
            self.left_arm_up = self.is_arm_up(results.pose_landmarks.landmark, "left")
            self.right_arm_bent = self.is_arm_bent(results.pose_landmarks.landmark, "right")
            self.left_arm_bent = self.is_arm_bent(results.pose_landmarks.landmark, "left")

            self.run_loop()


        
        # Display debug info
        cv2.putText(frame, f"Right Arm: {'UP' if self.right_arm_up else 'DOWN'} | {'BENT' if self.right_arm_bent else 'STRAIGHT'}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Left Arm: {'UP' if self.left_arm_up else 'DOWN'} | {'BENT' if self.left_arm_bent else 'STRAIGHT'}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Arm Gesture Control', frame)
        cv2.waitKey(1)

    def run_loop(self):
        # Read all available characters from stdin without blocking indefinitely
        char = ' '
        while select.select([sys.stdin], [], [], 0.001)[0]: # 1ms timeout for select
            try:
                char = sys.stdin.read(1)
                if char:
                    continue
                else: # EOF
                    self.shutdown_procedure()
                    return
            except Exception as e:
                self.get_logger().error(f"Error reading from stdin: {e}")
                self.shutdown_procedure()
                return
            
        if char == ESC_KEY:
            self.get_logger().info("ESC pressed, shutting down...")
            self.shutdown_procedure()
            return
        
        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons = [0] * 11
        if 1 < self.step <= 20:
            char = 's'
        elif 50 < self.step <=51:
            char = 'r'
        elif 75 < self.step <=80:
            axes[4] = 0.6
        elif 95 < self.step <=100:
            char = 'c'

        if self.step > 100:
            if self.right_arm_up and not self.left_arm_up and not self.right_arm_bent:
                char = 'w'
            elif self.left_arm_up and not self.right_arm_up and not self.left_arm_bent:
                char = 'z'
            elif self.right_arm_bent and self.right_arm_up and not self.left_arm_up and not self.left_arm_bent:
                char = 'd'
            elif self.left_arm_bent and self.left_arm_up and not self.right_arm_up and not self.right_arm_bent:
                char = 'a'
        
        if char == 's':
            buttons[0] = 1
            axes = axes

        if char == 'r':
            buttons[6] = 1

        if char == 'i':
            buttons[7] = 1

        if char == 'c':
            buttons[2] = 1

        if char == 'w':
            axes[4] = 0.7
            axes[0] = 0.1

        if char == 'd':
            axes[4] = 0.6
            axes[0] = 0.3

        if char == 'a':
            axes[4] = 0.6
            axes[0] = -0.3
        
        if char == 'z':
            axes[4] = -0.3


        self.current_joy_msg.axes = list(axes)
        self.current_joy_msg.buttons = list(buttons)
        
        self.publisher_.publish(self.current_joy_msg)
        self.step += 1

    
    def shutdown_procedure(self):
        self.get_logger().info("Restoring terminal settings and shutting down...")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        

    def destroy_node(self):
        self.get_logger().info("KeyboardTeleopJoy node is being destroyed.")
        self.shutdown_procedure()
        super().destroy_node()

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    print("Main function started")
    rclpy.init(args=args)
    keyboard_teleop_joy_node = None
    try:
        print("Main function started----try function")
        keyboard_teleop_joy_node = KeyboardTeleopJoy()
        rclpy.spin(keyboard_teleop_joy_node)
    except KeyboardInterrupt:
        keyboard_teleop_joy_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    except Exception as e:
        if keyboard_teleop_joy_node:
            keyboard_teleop_joy_node.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"Unhandled exception before node init: {e}")
    finally:
        if keyboard_teleop_joy_node and rclpy.ok():
            keyboard_teleop_joy_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if keyboard_teleop_joy_node and hasattr(keyboard_teleop_joy_node, 'settings'):
             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keyboard_teleop_joy_node.settings)
        print("Exited cleanly.")

if __name__ == '__main__':
    main()

