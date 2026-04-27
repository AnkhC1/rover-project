#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math, time

class MoveNode(Node):
    def __init__(self):
        super().__init__('calibration_move')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear_speed, angular_speed, duration):
     msg = Twist()
     msg.linear.x = linear_speed
     msg.angular.z = angular_speed
     self.pub.publish(msg)  # send once
     time.sleep(duration)   # wait
     self.pub.publish(Twist())  # stop
     
    def move_distance(self, distance, speed=0.5):
           self.move(speed, 0.0, distance / speed)

    def spin_angle(self, degrees, speed=0.5):
        radians = math.radians(degrees)
        self.move(0.0, speed, abs(radians) / speed)

def main():
    rclpy.init()
    node = MoveNode()
    time.sleep(0.5)  # give publisher time to connect
    node.spin_angle(1080.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
