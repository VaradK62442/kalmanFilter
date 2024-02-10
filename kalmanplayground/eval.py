'''
Subscribes to function pub, function sub, kalman filter.
Writes all three measurements to a file.
'''

import rclpy
from rclpy.node import Node

import os

from std_msgs.msg import String

class Eval(Node):

    def __init__(self):
        # delete files if they exist
        self.file_list = ["function_values.txt", "measurement_values.txt", "kalman_measurement_values.txt"]
        for file in self.file_list:
            if os.path.isfile("./" + file): os.remove(file) # definitely safe

        super().__init__('function_subscriber')
        self.subscription = self.create_subscription(
            String, 'function_value',
            self.set_function_val, 10
        )
        self.subscription = self.create_subscription(
            String, 'measurement', 
            self.set_measurement, 10
        )
        self.subscription = self.create_subscription(
            String, 'kalman_measurement',
            self.set_kalman, 10
        )


    def write_to_file(self, filename, msg):
        with open(filename, 'a') as f:
            f.write(msg.data.replace("[", "").replace("]", "") + "\n")


    def set_function_val(self, function_value):
        self.write_to_file(self.file_list[0], function_value)

    def set_measurement(self, measurement_value):
        self.write_to_file(self.file_list[1], measurement_value)

    def set_kalman(self, kalman_measurement):
        self.write_to_file(self.file_list[2], kalman_measurement)
        


def main(args=None):
    rclpy.init(args=args)

    eval = Eval()

    rclpy.spin(eval)

    eval.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()