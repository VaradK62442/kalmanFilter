'''
Publishes a certain function at a given time t.
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import numpy as np


class FunctionPublisher(Node):
    
    def __init__(self, function="sin"):
        super().__init__('function_publisher')
        self.publisher = self.create_publisher(
            String, 'function_value', 10
        )
        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.x = 0 # used as input to function
        self.function = function


    def timer_callback(self):
        msg = String()
        functionValue = self.get_function_value()
        msg.data = str(functionValue)
        
        self.publisher.publish(msg)
        # self.get_logger().info(f"Func Publishing: {msg}")
        self.x += self.timer_period


    def get_function_value(self):
        match self.function:
            case "sin": # also default
                return np.sin(self.x)
            case "cos":
                return np.cos(self.x)
            case "tan":
                return np.tan(self.x)
            case "x":
                return self.x
            case "x^2":
                return self.x * self.x
            case "x^3":
                return self.x ** 3
            case "exp":
                return np.e ** self.x


def main(args=None):
    rclpy.init(args=args)

    function_publisher = FunctionPublisher()

    rclpy.spin(function_publisher)

    function_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()