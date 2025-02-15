#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class TeleopTurtlebot(Node):
    def __init__(self):
        super().__init__("turtle_bot_teleop")
        self.publisher = self.create_publisher(Twist, 'turtlebot_cmdVel', 10)
        self.speed = float(input("VLIN: "))
        self.turn = float(input("VANG: "))
        self.run()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.get_key()
            if key == 'w':
                twist.linear.x = self.speed
            elif key == 's':
                twist.linear.x = -self.speed
            elif key == 'a':
                twist.angular.z = self.turn
            elif key == 'd':
                twist.angular.z = -self.turn
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.publisher.publish(twist)
            if key == 'q':
                break

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtlebot()
    rclpy.spin(node)
    rclpy.shutdown()
