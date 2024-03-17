#!/usr/bin/env python3

import os
import rospy
import socket
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from math import pi, radians
import time

TURN_FORWARD_DISTANCE = 0.20
FORWARD_DISTANCE = 0.25
BACKWARD_DISTANCE = 0.10
TURN_ANGLE = radians(90)
FORWARD_VELOCITY = 0.3
TURN_OMEGA = 4.0


class TwistControlNode(DTROS):
    def __init__(self, node_name):
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        print(f"TwistControlNode initialized. Publishing to topic: {twist_topic}")

    def forward(self):
        print("Executing forward behavior")

        time = FORWARD_DISTANCE / FORWARD_VELOCITY
        
        message = Twist2DStamped(v=FORWARD_VELOCITY, omega=0.0)
        self._publish_for_duration(message, time)
        print("Forward behavior completed")
        self.stop()

    def backward(self):
        print("Executing backward behavior")
        time = BACKWARD_DISTANCE / abs(FORWARD_VELOCITY)
        
        message = Twist2DStamped(v=-FORWARD_VELOCITY, omega=0.0)
        self._publish_for_duration(message, time)
        print("Backward behavior completed")
        self.stop()

    def right(self):
        print("Executing right behavior")
        self._turn(-TURN_ANGLE)  # 90 degrees (pi/2 radians)
        self.stop()
        rospy.sleep(1)
        self._forward(TURN_FORWARD_DISTANCE) 
        print("Right behavior completed")
        self.stop()

    def left(self):
        print("Executing left behavior")
        self._turn(TURN_ANGLE)  # -90 degrees (-pi/2 radians)
        self.stop()
        rospy.sleep(1)
        self._forward(TURN_FORWARD_DISTANCE)  # 4 cm
        print("Left behavior completed")
        self.stop()

    def _turn(self, angle):
        print(f"Turning by {angle} radians")
        omega = TURN_OMEGA
        time = abs(angle) / omega
        message = Twist2DStamped(v=0.0, omega=omega if angle > 0 else -omega)
        self._publish_for_duration(message, time)
        print("Turning completed")

    def _forward(self, distance):
        print(f"Moving forward by {distance} meters")
        velocity = FORWARD_VELOCITY  
        time = distance / velocity
        message = Twist2DStamped(v=velocity, omega=0.0)
        self._publish_for_duration(message, time)
        print("Forward movement completed")

    def stop(self):
        print("Executing stop behavior")
        message = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(message)
        print("Stop behavior completed")

    def _publish_for_duration(self, message, duration):
        print(f"Publishing message for {duration} seconds")
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        
        while rospy.Time.now() - start_time < rospy.Duration(duration) and not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()
        print("Publishing completed")

    def run(self):
        host = '0.0.0.0'  # Change to the desired host IP address
        port = 12345  # Change to the desired port number

        print(f"Starting server on {host}:{port}")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen()

            while not rospy.is_shutdown():
                print("Waiting for a connection...")
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024).decode().strip()
                        if not data:
                            break
                        print(f"Received command: {data}")
                        self.process_command(data)

    def process_command(self, command):
        print(f"Processing command: {command}")
        if command == 'f':
            self.forward()
        elif command == 'b':
            self.backward()
        elif command == 'r':
            self.right()
        elif command == 'l':
            self.left()
        elif command == 's':
            self.stop()
        else:
            print(f"Unknown command: {command}")

    def on_shutdown(self):
        print("Shutting down TwistControlNode")
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)
        print("Shutdown complete")

if __name__ == '__main__':
    print("Starting TwistControlNode")
    node = TwistControlNode(node_name='twist_control_node')
    node.run()