#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pcl_msgs.srv import UpdateFilename
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import json

class TurtleBotPlayerNode(Node):

    def __init__(self):

        super().__init__("turtle_bot_player")
        self.turtlebot_cmdVel = self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.turtlebot_simTime = self.create_subscription(Float32, "/simulationTime", self.update_turtle_simTime, 10)

        self.srv = self.create_service(UpdateFilename, '/turtlebot_player_service', self.handle_file_request)
        self.get_logger().info('Servicio "get_file_path" listo...')


    def handle_file_request(self, request, response):

        file_path = request.filename
        self.get_logger().info(f'Recibido file_path: {file_path}')
        
        try:
            with open(file_path, 'r') as file:
                self.data = json.load(file) 
                response.success = True
        except Exception as e:
            response.success = False
        
        return response
    



def main(args=None):

    try:
        rclpy.init(args=args)
        node = TurtleBotPlayerNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:

        if node is not None:
            print(f" pressed Player Finishing...\n")
    
    finally:

        print(f"Player Finished Successfuly :) \n")


if __name__ == '__main__':
    main()
