import numpy as np

def read_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y, _ = map(float, line.strip().split())
            data.append((x, y))
    return np.array(data)

def calculate_deviation(truth_data, odom_data):
    deviations = []
    for truth, odom in zip(truth_data, odom_data):
        deviation = np.sqrt((odom[0] - truth[0])**2 + (odom[1] - truth[1])**2)
        deviations.append(deviation)
    deviations = np.array(deviations)
    mean = np.mean(deviations)
    variance = np.var(deviations)
    return mean, variance

def main():
    truth_data = read_file('base_truth.txt')
    odom_data = read_file('odom.txt')
    odom_combined_data = read_file('odom_combined.txt')
    odom_combined2_data = read_file('odom_combined2.txt')

    # Calculate deviation for odom
    mean_odom, var_odom = calculate_deviation(truth_data, odom_data)
    print("Odom deviation - Mean:", mean_odom, "Variance:", var_odom)

    # Calculate deviation for odom_combined
    mean_combined, var_combined = calculate_deviation(truth_data, odom_combined_data)
    print("Odom combined deviation - Mean:", mean_combined, "Variance:", var_combined)

    # Calculate deviation for odom_combined2
    mean_combined2, var_combined2 = calculate_deviation(truth_data, odom_combined2_data)
    print("Odom combined2 deviation - Mean:", mean_combined2, "Variance:", var_combined2)

if __name__ == "__main__":
    main()
