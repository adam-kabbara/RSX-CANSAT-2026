from airfoil_performance_estimator import parse_xflr5_polar
import matplotlib.pyplot as plt


if __name__ == "__main__":
    data = parse_xflr5_polar("./polars/naca644421-ilauto/T1_Re450000_M0_N9.txt")
    plt.plot(data["columns"]["alpha"], data["columns"]["CL"])
    data = parse_xflr5_polar("./polars/Fx63-137/T1_Re0.060_M0.00_N9.0.txt")
    plt.plot(data["columns"]["alpha"], data["columns"]["CL"])
    plt.legend(["Auto Script", "XFLR5"])
    plt.show()
