import matplotlib.pyplot as plt
import matplotlib.font_manager

def read_coordinates_from_file(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        coordinates = [(float(line.split()[0]), float(line.split()[1])) for line in lines]
    return coordinates

def plot_odom_trajectory(odom_coordinates, title, color):
    x = [coord[0]/4*5 for coord in odom_coordinates]
    y = [coord[1]/4*5 for coord in odom_coordinates]
    plt.plot(x, y, marker=',', label=title, color=color, linewidth=2)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    # plt.title('Odom Trajectory')
    plt.legend()
    
def plot_odom5x_trajectory(odom_coordinates, title, color, linewidth=2):
    x = [coord[0] for coord in odom_coordinates]
    y = [coord[1] for coord in odom_coordinates]
    plt.plot(x, y, marker=',', label=title, color=color, linewidth=linewidth)
    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.title('Odom Trajectory')
    plt.axis('equal')
    plt.legend()

if __name__ == "__main__":
    base_truth_file = 'base_truth.txt'
    odom_file = 'odom.txt'
    odom_combined_file = 'odom_combined.txt'
    odom_combined_file2 = 'odom_combined2.txt'

    base_truth_coordinates = read_coordinates_from_file(base_truth_file)
    odom_coordinates = read_coordinates_from_file(odom_file)
    odom_combined_coordinates = read_coordinates_from_file(odom_combined_file)
    odom_combined_coordinates2 = read_coordinates_from_file(odom_combined_file2)

    plt.figure()
    # plt.rcParams['font.family'] = ['Noto Sans CJK SC', 'sans-serif']
    # plot_odom5x_trajectory(base_truth_coordinates, '基准里程计数据', 'red')
    # plot_odom5x_trajectory(odom_coordinates, '原始里程计数据', 'orange')
    # plot_odom5x_trajectory(odom_combined_coordinates, '里程计和 IMU 融合数据', 'green')
    # plot_odom5x_trajectory(odom_combined_coordinates2, '仅使用里程计融合数据', 'blue')
    plot_odom5x_trajectory(base_truth_coordinates, 'Base Truth', 'red')
    # plot_odom5x_trajectory(odom_coordinates, 'Raw Odom', 'orange')
    # plot_odom5x_trajectory(odom_combined_coordinates, 'EKF Fusion with Odom and IMU', 'green')
    plot_odom5x_trajectory(odom_combined_coordinates2, 'EKF Fusion with Odom', 'blue')
    plt.grid(True)
    plt.show()
