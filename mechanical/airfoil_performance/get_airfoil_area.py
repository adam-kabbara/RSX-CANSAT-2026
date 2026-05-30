import numpy as np

def read_selig_dat(filename):
    """
    Reads a Selig-format airfoil .dat file.
    Returns an Nx2 numpy array of (x, y) points.
    """
    points = []

    with open(filename, "r") as f:
        lines = f.readlines()

    # Skip first line (airfoil name)
    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) == 2:
            x, y = map(float, parts)
            points.append([x, y])

    return np.array(points)


def polygon_area(points):
    """
    Computes area of a closed polygon using the shoelace formula.
    """
    x = points[:, 0]
    y = points[:, 1]

    # Close the polygon if needed
    if not np.allclose(points[0], points[-1]):
        x = np.append(x, x[0])
        y = np.append(y, y[0])

    area = 0.5 * abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))
    return area


def get_airfoil_area(dat_file: str):
    pts = read_selig_dat(dat_file)
    return polygon_area(pts)


if __name__ == "__main__":
    filename = "airfoil.dat"  # replace with your file
    airfoil_points = read_selig_dat(filename)
    area = polygon_area(airfoil_points)

    print(f"Airfoil area: {area}")


