#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from candle_ros2.msg import MotionCmd
from pynput import keyboard

# -------- CONFIGURATION --------
MAX_SPEED   = 2.0
NODE_PREFIX = "md/"

# Set your actual motor CAN IDs here
DEVICE_IDS  = [180, 181, 182, 183]

# -------- GLOBAL STATE --------
running      = True
keys_pressed = set()


# -------- ROS2 NODE --------

class CandleTeleopNode(Node):
    def __init__(self, device_ids: list):
        super().__init__("candle_keyboard_teleop")
        self.device_ids = device_ids
        self._pub = self.create_publisher(MotionCmd, NODE_PREFIX + "motion_command", 10)

    def publish_velocities(self, left_vel, right_vel):
        msg = MotionCmd()
        msg.device_ids      = self.device_ids
        msg.target_position = [0.0] * len(self.device_ids)
        msg.target_torque   = [0.0] * len(self.device_ids)
        msg.target_velocity = [
            float(left_vel if i % 2 == 0 else right_vel) * (MAX_SPEED / 128)
            for i in range(len(self.device_ids))
        ]
        self._pub.publish(msg)

    def stop(self):
        self.publish_velocities(0, 0)
        self.get_logger().info("Motors stopped.")


# -------- KEYBOARD LISTENER --------

def on_press(key):
    keys_pressed.add(key)

def on_release(key):
    keys_pressed.discard(key)
    if key == keyboard.Key.esc:
        global running
        running = False


# -------- MAIN LOOP --------

def run_receiver(ros_node: CandleTeleopNode):
    print("Teleop running. Use arrow keys to drive, ESC to quit.")

    try:
        while running and rclpy.ok():

            forward  = keyboard.Key.up    in keys_pressed
            backward = keyboard.Key.down  in keys_pressed
            left     = keyboard.Key.left  in keys_pressed
            right    = keyboard.Key.right in keys_pressed

            fwd_speed       = 128  if forward  else 0
            bwd_speed       = -128 if backward else 0
            rotation_speed  = 128
            linear_velocity = fwd_speed + bwd_speed

            if left and not right:
                left_speed, right_speed = -rotation_speed, -rotation_speed
            elif right and not left:
                left_speed, right_speed = rotation_speed, rotation_speed
            else:
                left_speed, right_speed = linear_velocity, -linear_velocity

            ros_node.publish_velocities(left_speed, right_speed)

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        print("Shutting down.")
    finally:
        ros_node.stop()


# -------- ENTRY POINT --------

def main():
    rclpy.init()
    ros_node = CandleTeleopNode(device_ids=DEVICE_IDS)

    # ROS2 spin in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Keyboard listener in background thread
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    run_receiver(ros_node)

    listener.stop()
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()