#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys, os
import termios, select
import fcntl



class turtle_bot_teleop_node(Node):
    def __init__(self):
        super().__init__("turtle_bot_teleop")
        self.turtlebot_cmdVel = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
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

        self.sys_settings = sys.stdin.fileno()
        newattr = termios.tcgetattr(self.sys_settings)
        newattr[3] = newattr[3] & ~termios.ICANON
        newattr[3] = newattr[3] & ~termios.ECHO
        termios.tcsetattr(self.sys_settings, termios.TCSANOW, newattr)

        self.oldterm = termios.tcgetattr(self.sys_settings)
        self.oldflags = fcntl.fcntl(self.sys_settings, fcntl.F_GETFL)
        fcntl.fcntl(self.sys_settings, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

        self.angular_vel = angular_vel
        self.lineal_vel = lineal_vel
        self.key = None

        while (self.key != "q"):
            self.getKey()
            msg = Twist()
            if self.key == "w":
                msg.linear.x = self.lineal_vel
            elif self.key == "s":
                msg.linear.x = -self.lineal_vel
            elif self.key == "d":
                msg.angular.z = self.angular_vel
            elif self.key == "a":
                msg.angular.z = -self.angular_vel
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.turtlebot_cmdVel.publish(msg)
        if self.key == "q":
            self.cleanup_terminal()
            self.destroy_node()
        
    def getKey(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            #tty.setraw(sys.stdin.fileno())
            k = sys.stdin.read(1)
            
            if k == '\x1b':
                kk = sys.stdin.read(2)
                match kk:
                    case '[A':
                        k = "w"  # Flecha arriba
                    case '[B':
                        k = "s"  # Flecha abajo
                    case '[C':
                        k = "d"  # Flecha derecha
                    case '[D':
                        k = "a"  # Flecha izquierda
                    case _:
                        k = None
            self.key = k
        else:
            # Si no se presionó ninguna tecla, el valor de key es None
            self.key = None

    def cleanup_terminal(self):
        # Restaurar los atributos originales de la terminal
        termios.tcsetattr(self.sys_settings, termios.TCSAFLUSH, self.oldterm)
        fcntl.fcntl(self.sys_settings, fcntl.F_SETFL, self.oldflags)
        os.system('stty sane')

def main(args=None):
    rclpy.init(args=args)
    node = turtle_bot_teleop_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        pass
    finally:
        node.cleanup_terminal()
        rclpy.shutdown()