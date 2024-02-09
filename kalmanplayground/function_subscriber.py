'''
Subscribes to function publisher.
"Measures" published message with some noise.
Publishes measurement.
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import numpy as np


class FunctionSubscriber(Node):

    def __init__(self, std=0.1):
        super().__init__('function_subscriber')
        self.subscription = self.create_subscription(
            String, 'function_value', 
            self.listener_callback, 10
        )
        self.publisher = self.create_publisher(
            String, 'measurement', 10
        )
        self.std = std # for noise function


    def get_noise(self):
        return np.random.normal(
            0, # mean
            self.std, # std
            1 # number of elements
        )
    

    def listener_callback(self, function_value):
        msg = String()
        measurement = float(function_value.data) + self.get_noise()
        msg.data = str(measurement)
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Meas Publishing: {msg}")


def main(args=None):
    rclpy.init(args=args)

    function_subscriber = FunctionSubscriber(std=0.05)

    rclpy.spin(function_subscriber)

    function_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()