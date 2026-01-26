# plot aerfoil given a file

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def plot_airfoil(x_coords, y_coords, name):
    """
    Plots the airfoil shape given x and y coordinates.

    Args:
        x_coords (list or np.array): X coordinates of the airfoil.
        y_coords (list or np.array): Y coordinates of the airfoil.
        title (str): Title of the plot.
    """
    # read txt file, x y seperated "  " first line is the aerfoil name
    plt.figure(figsize=(10, 5))
    plt.plot(x_coords, y_coords, marker='o')
    plt.title(f'Airfoil Shape from {name}')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def get_points_from_file(file_path):
    data = pd.read_csv(file_path, delim_whitespace=True, skiprows=1, names=['x', 'y'])
    x_coords = data['x'].values
    y_coords = data['y'].values
    # get the first line of the file for the name
    with open(file_path, 'r') as f:
        name = f.readline().strip()
    return x_coords, y_coords, name

def fix_airfoil_coordinates(x_coords, y_coords):
    # some airfoils are not circular, the have a defined start and end point. fix that by connecting the end to the start
    if x_coords[0] != x_coords[-1] or y_coords[0] != y_coords[-1]:
        x_coords = np.append(x_coords, x_coords[0])
        y_coords = np.append(y_coords, y_coords[0])
    return x_coords, y_coords


if __name__ == "__main__":
    # example usage
    x, y, name = get_points_from_file(r'mechanical\airfoil_performance\seligdatfile.txt')
    plot_airfoil(x, y, name)
    x, y = fix_airfoil_coordinates(x, y)
    plot_airfoil(x, y, name)

    # save into a new file
    with open(r'mechanical\airfoil_performance\seligdatfile_fixed.txt', 'w') as f:
        f.write(f"{name}\n")
        for xi, yi in zip(x, y):
            f.write(f"{xi} {yi}\n")

    
