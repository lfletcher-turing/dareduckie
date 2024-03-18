#!/usr/bin/env python3

import os
import rospy
import socket
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Imu
import math
import json
import copy

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        encoder_topic_left = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        encoder_topic_right = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        imu_topic = f"/{vehicle_name}/imu_node/data"

        with open('/data/config/dareduckie.json', 'r') as file:
            constants = json.load(file)

        # print the constants
        print(constants)

        self.initial_left_encoder_ticks = None
        self.initial_right_encoder_ticks = None
        self.current_left_encoder_ticks = 0
        self.current_right_encoder_ticks = 0
        self.orientation_imu = 0
        self.orientation_encoders = 0
        self.orientation_filtered = 0.0
        self.current_distance = 0.0
        self.current_angle_encoders = 0.0
        self.last_imu_update_time = None

        self.wheel_radius = constants['wheel_radius']   # Radius of the wheel in meters
        self.wheel_separation = constants['wheel_separation']  # Distance between the wheels in meters
        self.encoder_resolution = constants['encoder_resolution']  # Number of encoder ticks per revolution
        self.kp_distance = constants['kp_distance']
        self.kd_distance = constants['kd_distance']
        self.kp_angle = constants['kp_angle']
        self.kd_angle = constants['kd_angle']
        self.min_throttle_magnitude = constants['min_throttle_magnitude']
        self.max_throttle_magnitude = constants['max_throttle_magnitude']
        self.forward_distance = constants['forward_distance']
        self.backward_distance = constants['backward_distance']
        self.turn_angle = math.radians(constants['turn_angle']) # Angle to turn 
        self.turn_then_forward_distance = constants['turn_then_forward_distance'] # Distance to move forward after turning
        self.alpha = constants['alpha'] # Weight for the complementary filter

        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        rospy.Subscriber(encoder_topic_left, WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber(encoder_topic_right, WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber(imu_topic, Imu, self.imu_callback)

    def left_encoder_callback(self, encoder_msg):
        self.current_left_encoder_ticks = encoder_msg.data
        if self.initial_left_encoder_ticks is None:
            self.initial_left_encoder_ticks = self.current_left_encoder_ticks

    def right_encoder_callback(self, encoder_msg):
        self.current_right_encoder_ticks = encoder_msg.data
        if self.initial_right_encoder_ticks is None:
            self.initial_right_encoder_ticks = self.current_right_encoder_ticks

    def imu_callback(self, imu_msg):
        # Update the angular velocity
        self.angular_velocity_z = imu_msg.angular_velocity.z


    def update_orientation(self):
        # Update the orientation based on the encoder
        if self.last_imu_update_time is None:
            self.orientation_imu = 0
            self.last_imu_update_time = rospy.get_time()
            self.orientation_filtered = 0
            return

        encoder_orientation = self.current_angle_encoders
        # print the encoder orientation in degrees
        print(f"Encoder orientation: {math.degrees(encoder_orientation)}")

        
        current_time = rospy.get_time()

        # Calculate the time difference since the last IMU update
        dt = current_time - self.last_imu_update_time

        # Update the orientation based on the angular velocity from the IMU
        angular_velocity_z = self.angular_velocity_z
        self.orientation_imu += angular_velocity_z * dt

        # Update the last IMU update time
        self.last_imu_update_time = current_time

        imu_orientation = self.orientation_imu
        # print the imu orientation in degrees
        print(f"IMU orientation: {math.degrees(imu_orientation)}")

        # Combine the orientations from the encoders and IMU
        self.orientation_filtered = self.alpha * imu_orientation + (1 - self.alpha) * encoder_orientation

    
    def calculate_encoder_distances(self):
        left_encoder = copy.copy(self.current_left_encoder_ticks)
        right_encoder = copy.copy(self.current_right_encoder_ticks)
        left_distance = (2 * math.pi * self.wheel_radius * (left_encoder - self.initial_left_encoder_ticks)) / self.encoder_resolution
        right_distance = (2 * math.pi * self.wheel_radius * (right_encoder - self.initial_right_encoder_ticks)) / self.encoder_resolution
        self.current_distance = (left_distance + right_distance) / 2
        self.current_angle_encoders = (right_distance - left_distance) / self.wheel_separation

    
    def store_initial_encoder_values(self):
        self.initial_left_encoder_ticks = copy.copy(self.current_left_encoder_ticks)
        self.initial_right_encoder_ticks = copy.copy(self.current_right_encoder_ticks)

    def move_forward(self, distance = None):
        if distance is None:
            distance = self.forward_distance
        self.move(distance, 0.0)
        self.stop()
        rospy.sleep(1)
        self.sensor_reset()
        print("Forward movement completed")

    def move_backward(self, distance=None):
        if distance is None:
            distance = self.backward_distance
        self.move(-distance, 0.0)
        self.stop()
        rospy.sleep(1)
        self.sensor_reset()
        print("Backward movement completed")

    def turn_left(self, angle = None):
        if angle is None:
            angle = self.turn_angle
        self.move(0.0, angle)
        self.stop()
        print("Left turn completed")
        rospy.sleep(1)
        self.sensor_reset()
        self.move(self.turn_then_forward_distance, 0.0)
        self.stop()
        rospy.sleep(1)
        self.sensor_reset()
        print("Forward movement completed")

    def turn_right(self, angle = None):
        if angle is None:
            angle = self.turn_angle
        self.move(0.0, -angle)
        print("Right turn completed")
        self.stop()
        rospy.sleep(1)
        self.sensor_reset()
        self.move(self.turn_then_forward_distance, 0.0)
        rospy.sleep(1)
        self.sensor_reset()
        print("Forward movement completed")

    def sensor_reset(self):
        self.store_initial_encoder_values()
        self.last_imu_update_time = None
        self.orientation_filtered = 0.0
        self.orientation_imu = 0.0

    def move(self, desired_distance, desired_angle):
        rate = rospy.Rate(20)  # Control loop rate of 20 Hz
        previous_distance_error = 0.0
        previous_angle_error = 0.0

        if desired_angle != 0:
            print(f"Turning by {math.degrees(desired_angle)} degrees")

        while not rospy.is_shutdown():

            self.calculate_encoder_distances()
            self.update_orientation()
            current_distance = self.current_distance
            current_angle = self.orientation_filtered

            # print(f"current distance: {current_distance}")
            # print(f"current angle(deg): {math.degrees(current_angle)}")

            # Calculate the error and apply PD control
            distance_error = desired_distance - current_distance
            angle_error = desired_angle - current_angle
            # print(f"Angle error: {angle_error}")
            distance_output = self.kp_distance * distance_error + self.kd_distance * (distance_error - previous_distance_error)
            angle_output = self.kp_angle * angle_error + self.kd_angle * (angle_error - previous_angle_error)

            # Calculate the wheel velocities based on the control output
            vel_left = distance_output - angle_output
            vel_right = distance_output + angle_output


                    # Apply minimum throttle magnitude
            if vel_left > 0:
                vel_left = max(min(vel_left, self.max_throttle_magnitude), self.min_throttle_magnitude)
            else:
                vel_left = min(max(vel_left, -self.max_throttle_magnitude), -self.min_throttle_magnitude)
            if vel_right > 0:
                vel_right = max(min(vel_right, self.max_throttle_magnitude), self.min_throttle_magnitude)
            else:
                vel_right = min(max(vel_right, -self.max_throttle_magnitude), -self.min_throttle_magnitude)

            # Publish the wheel velocities
            message = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
            self._publisher.publish(message)

            previous_distance_error = distance_error
            previous_angle_error = angle_error
            rate.sleep()

            # Check if the desired distance and angle are reached
            if abs(distance_error) < 0.025 and abs(angle_error) < 0.1:
                break

        # Stop the wheels after reaching the desired distance and angle
        self.stop()

    def stop(self):
        stop_message = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop_message)

    # def run(self):
    #     host = '0.0.0.0'  # Change to the desired host IP address
    #     port = 12345  # Change to the desired port number

    #     print(f"Starting server on {host}:{port}")
    #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #         s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #         s.bind((host, port))
    #         s.listen()

    #         while not rospy.is_shutdown():
    #             print("Waiting for a connection...")
    #             try:
    #                 conn, addr = s.accept()
    #             except socket.timeout:
    #                 continue  # Continue waiting for a connection if a timeout occurs

    #             print(f"Connected by {addr}")
                
    #             try:
    #                 while not rospy.is_shutdown():
    #                     data = conn.recv(1024).decode().strip()
    #                     if not data:
    #                         break
    #                     print(f"Received command: {data}")
    #                     self.process_command(data)
    #             except socket.error as e:
    #                 print(f"Socket error: {e}")
    #             finally:
    #                 conn.close()
    #                 print(f"Connection closed by {addr}")

    def run(self):
        host = '0.0.0.0'  # Change to the desired host IP address
        port = 12345  # Change to the desired port number

        print(f"Starting server on {host}:{port}")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()
            s.settimeout(1.0)

            while not rospy.is_shutdown():
                print("Waiting for a connection...")
                try:
                    conn, addr = s.accept()
                except socket.timeout:
                    continue
                print(f"Connected by {addr}")
            
                try:
                    while not rospy.is_shutdown():
                        data = conn.recv(1024).decode().strip()
                        if not data:
                            break
                        print(f"Received command: {data}")
                        self.process_command(data)
                except socket.error as e:
                    print(f"Socket error: {e}")
                finally:
                    conn.close()
                    print(f"Connection closed by {addr}")


    

    def process_command(self, command):
        print(f"Processing command: {command}")
        if command == 'f':
            self.move_forward()
        elif command == 'b':
            self.move_backward()
        elif command == 'r':
            self.turn_right()
        elif command == 'l':
            self.turn_left()
        elif command == 's':
            self.stop()
        else:
            print(f"Unknown command: {command}")

    def on_shutdown(self):
        self.stop()

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
    rospy.spin()