#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class turtle_bot_teleop_node(Node):
    def __init__(self):
        super().__init__("turtle_bot_teleop")
        self.turtlebot_cmdVel = self.create_publisher(Twist, "/turtlebot_cmdVel",10)
        angular_vel = None
        lineal_vel = None

        while (angular_vel == None or lineal_vel == None):
            if not(isinstance(angular_vel, float)):
                try:
                    angular_vel = float(input(f'Ingrese la velocidad angular del robot: \n -> '))
                except:
                    self.get_logger().info(f'La velocidad angular debe ser tipo float')
            if not(isinstance(lineal_vel, float)):
                try:
                    lineal_vel = float(input(f'Ingrese la velocidad lineal del robot: \n -> '))
                except:
                    self.get_logger().info(f'La velocidad lineal debe ser tipo float')
        self.angular_vel = angular_vel
        self.lineal_vel = lineal_vel
        self.timer = self.create_timer(1, self.send_velocity_command)
        self.get_logger().info(f'This node its working :)')

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = self.lineal_vel
        msg.angular.z = self.angular_vel
        self.turtlebot_cmdVel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = turtle_bot_teleop_node()
    rclpy.spin(node)
    rclpy.shutdown()