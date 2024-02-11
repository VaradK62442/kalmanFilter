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
        self.position_prediction = 0 # start prediction at 0
        self.velocity_prediction = 0
        self.accel_prediction = 0


    def listener_callback(self, measurement_data):
        msg = String()
        measurement = measurement_data.data.replace("[", "").replace("]", "")
        self.n += 1
        # msg.data = self.alpha_beta_filter(measurement)
        msg.data = self.alpha_beta_gamma_filter(measurement)

        self.publisher.publish(msg)
        # self.get_logger().info(f"Kalm Publishing: {msg}")


    def alpha_beta_filter(self, measurement: str) -> str:
        # n = self.n
        z_n = float(measurement) # current measurement
        x_n_n1 = self.position_prediction # prior prediction
        x_dot_n_n1 = self.velocity_prediction
        alpha = 0.7
        beta = 0.05
        dt = 0.1 # change in time

        new_position = x_n_n1 + alpha * (z_n - x_n_n1)
        new_velocity = x_dot_n_n1 + beta * ((z_n - x_n_n1) / dt) 

        new_prediction = new_position + dt * new_velocity

        self.position_prediction = new_prediction
        return str(new_prediction)
    

    def alpha_beta_gamma_filter(self, measurement: str) -> str:
        # http://code.eng.buffalo.edu/tracking/papers/tenne_optimalFilter2000.pdf
        x_p = self.position_prediction # prediction at time k
        alpha = 0.7
        x_o = float(measurement) # observation at time k
        x_s = x_p + alpha * (x_o - x_p)

        v_p = self.velocity_prediction # vel prediction at k
        beta = 0.05
        T = 0.1 # ??? idk what this is (probably time delta)
        v_s = v_p + (beta / T) * (x_o - x_p)

        a_s = self.accel_prediction
        gamma = 0.5 # TODO: tune this
        a_s = a_s + (gamma / (2 * T * T)) * (x_o - x_p) # new a_s

        # new predictions
        x_p = x_s + T * v_s + (1 / 2) * T * T * a_s
        v_p = v_s + T * a_s

        # update predictions
        self.position_prediction = x_p
        self.velocity_prediction = v_p
        self.accel_prediction = a_s

        return str(self.position_prediction)


def main(args=None):
    rclpy.init(args=args)

    kalman_filter = KalmanFilter()

    rclpy.spin(kalman_filter)

    kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
