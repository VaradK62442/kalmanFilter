'''
Reads data in from file.
Produces plot of three measurements.
'''

import matplotlib.pyplot as plt
import numpy as np

def read_data_from_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            data.append(float(line.strip()))

    return data


def main():
    function_vals = read_data_from_file("function_values.txt")
    measurement_vals = read_data_from_file("measurement_values.txt")
    kalman_vals = read_data_from_file("kalman_measurement_values.txt")

    plt.figure()
    x_vals = list(range(len(function_vals)))

    plt.plot(x_vals, function_vals, label="Expected")
    plt.plot(x_vals, measurement_vals, label="Measured")
    plt.plot(x_vals, kalman_vals, label="Kalman")

    kalman_error = np.sqrt(np.mean(np.square(np.array(function_vals) - np.array(kalman_vals)),axis=0))
    measurement_error = np.sqrt(np.mean(np.square(np.array(function_vals) - np.array(measurement_vals)),axis=0))

    print(f"Kalman error: {kalman_error}\nMeasurement error: {measurement_error}")

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()