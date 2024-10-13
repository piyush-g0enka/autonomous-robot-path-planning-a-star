#!/usr/bin/env python3
# Importing ROS Client Library for Python
import rclpy


from turtlebot3_project3.turtlebot_controller_interface import (
    Controller,
)


def main(args=None):

    rclpy.init(args=args)  # Initializing the ROS client library
    controller_node = Controller("controller_node")
    rclpy.spin(controller_node)  # Keeping the node alive to listen for incoming data or events
    controller_node.destroy_node()  # Properly destroying the node once it's no longer needed
    rclpy.shutdown()  # Shutting down the ROS client library

if __name__ == "__main__":
    main()  # Executing the main function when the script is run