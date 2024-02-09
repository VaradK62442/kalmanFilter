'''
Recieves measurements.
Applies kalman filter.
Publishes new measurements.
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import numpy as np


class KalmanFilter(Node):

    def __init__(self, std=0.1):
        super().__init__('function_subscriber')
        self.subscription = self.create_subscription(
            String, 'measurement', 
            self.listener_callback, 10
        )
        self.publisher = self.create_publisher(
            String, 'kalman_measurement', 10
        )
        self.n = 0 # how many measurements
        self.prediction = 0 # start prediction at 0


    def listener_callback(self, measurement_data):
        msg = String()
        measurement = measurement_data.data.replace("[", "").replace("]", "")
        self.n += 1
        msg.data = self.alpha_beta_filter(measurement)

        self.publisher.publish(msg)
        self.get_logger().info(f"Kalm Publishing: {msg}")
        

    def alpha_beta_filter(self, measurement: str) -> str:
        n = self.n
        z_n = float(measurement) # current measurement
        x_n_n1 = self.prediction # prior prediction
        kalman_gain = 1/n

        new_prediction = x_n_n1 + kalman_gain * (z_n - x_n_n1)
        self.prediction = new_prediction

        return str(new_prediction)


def main(args=None):
    rclpy.init(args=args)

    kalman_filter = KalmanFilter()

    rclpy.spin(kalman_filter)

    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()