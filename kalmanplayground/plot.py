'''
Reads data in from file.
Produces plot of three measurements.
'''

import matplotlib.pyplot as plt

def read_data_from_file(filename):
    data = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            data.append(float(line.strip()))

    return data


def main():
    function_vals = read_data_from_file("function_vals.txt")
    measurement_vals = read_data_from_file("measurement_values.txt")
    kalman_vals = read_data_from_file("kalman_measurement_values.txt")

    plt.figure()
    x_vals = list(range(len(function_vals)))

    plt.plot(x_vals, function_vals, label="Expected")
    plt.plot(x_vals, measurement_vals, label="Measured")
    plt.plot(x_vals, kalman_vals, label="Kalman")

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()