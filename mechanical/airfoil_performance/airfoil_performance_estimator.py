import math
import numpy as np
import os
import matplotlib.pyplot as plt

AIRFOIL_NAME = "SD7037"
CHAR_LEN = 0.05 # coord len in meters
WING_LEN = 0.25 #*2 # 0.25m estimate wing len that fits in the container - *2 for folding design 
MASS = 0.3 # Half of target mass, as we are only studying one wing

G = 9.81
MU_AIR = 1.46e-5 # Kinematic viscosity of air @20C
RHO_AIR = 1.225 # Density of air @20C
WING_AREA = WING_LEN * CHAR_LEN # Assuming constant chord width (if we want to do variable we have to recompute Re as we move down the wing)
v_descent = 5

def reynolds_number(char_len, speed):
    re = char_len*speed/MU_AIR
    return re

def obtain_cl_cd(airfoil_table, v, alpha):
    re = reynolds_number(CHAR_LEN, v)
    closest_re = min(airfoil_table, key=lambda x:abs(x-re))
    closest_alpha = min(airfoil_table[closest_re]["columns"]["alpha"], key=lambda x: abs(x-alpha))
    alpha_index = airfoil_table[closest_re]["columns"]["alpha"].index(closest_alpha)
    return airfoil_table[closest_re]["columns"]["CL"][alpha_index], airfoil_table[closest_re]["columns"]["CD"][alpha_index]

def speed_solver(airfoil_table, alpha, wing_area, mass, air_density, v_init=12):
    E_descent = mass * G * v_descent
    v_prev = v_init
    v = 0
    while abs(v-v_prev) > 0.3:
        v_prev = v
        re = reynolds_number(CHAR_LEN, v_prev)
        closest_re = min(airfoil_table, key=lambda x:abs(x-re))
        closest_alpha = min(airfoil_table[closest_re]["columns"]["alpha"], key=lambda x: abs(x-alpha))
        alpha_index = airfoil_table[closest_re]["columns"]["alpha"].index(closest_alpha)
        v = math.cbrt((2*E_descent)/(airfoil_table[closest_re]["columns"]["CD"][alpha_index] * air_density * wing_area))
        print(abs(v-v_prev))
    return v


def get_lift_data(cl, wing_area, air_density, speed):
    return cl * air_density * ((speed**2)/2) * wing_area


def parse_xflr5_polar(filename):
    data = {
        "metadata": {
            "airfoil": None,
            "mach": None,
            "reynolds": None,
            "ncrit": None,
            "xtr_top": None,
            "xtr_bottom": None
        },
        "columns": {},
    }
    
    with open(filename, "r") as f:
        lines = [line.strip() for line in f if line.strip()]

    # Go through each line to extract metadata and table
    for i, line in enumerate(lines):
        if line.lower().startswith("calculated polar for"):
            data["metadata"]["airfoil"] = line.split(":")[-1].strip()
        
        elif line.lower().startswith("xtrf"):
            parts = line.split()
            data["metadata"]["xtr_top"] = float(parts[2])
            data["metadata"]["xtr_bottom"] = float(parts[4])
        
        elif line.lower().startswith("mach"):
            parts = line.replace("e", "E").split()
            data["metadata"]["mach"] = float(parts[2])
            data["metadata"]["reynolds"] = float(parts[5]) * 1e6
            data["metadata"]["ncrit"] = float(parts[-1])
        
        elif line.lower().startswith("alpha"):
            headers = line.split()
            
            # Initialize lists for each column
            for h in headers:
                data["columns"][h] = []
            
            # Start reading the numeric table after dashed line
            j = i + 2
            while j < len(lines):
                row = lines[j].split()
                try:
                    values = list(map(float, row))
                except ValueError:
                    break
                
                for h, v in zip(headers, values):
                    data["columns"][h].append(v)
                j += 1
            break

    return data



if __name__ == "__main__":
    alphas = np.linspace(0, 20, 100)
    airfoil_table = {}
    for polar in os.listdir(f".\\polars\\{AIRFOIL_NAME}"):
        parsed_polar = parse_xflr5_polar(os.path.join(f".\\polars\\{AIRFOIL_NAME}", polar))
        airfoil_table[parsed_polar["metadata"]["reynolds"]] = parsed_polar

    parameters = {"Alpha" : [], "Speed": [], "Re": [], "Cl": [], "Cd": [], "Cl/Cd": [], "Lift": []}
    units = {"Alpha" : "deg", "Speed": "m/s", "Re": "", "Cl": "", "Cd": "", "Cl/Cd": "", "Lift": "N"}
    for alpha in alphas:
        v = speed_solver(airfoil_table, alpha, WING_AREA, MASS, RHO_AIR)
        cl, cd = obtain_cl_cd(airfoil_table, v, alpha)
        v_horizontal = math.sqrt(v**2 - v_descent**2)
        print(f"Alpha (wing angle): {alpha}, Speed (diagonally x,y): {v}, Speed (x, horizontal): {v_horizontal}, Cl: {cl}, Cd: {cd}, Re: {reynolds_number(CHAR_LEN, v)}")
        lift = get_lift_data(cl, WING_AREA, RHO_AIR, v)
        parameters["Alpha"].append(alpha)
        parameters["Speed"].append(v)
        parameters["Re"].append(reynolds_number(CHAR_LEN, v))
        parameters["Cl"].append(cl)
        parameters["Cd"].append(cd)
        parameters["Cl/Cd"].append(cl/cd)
        parameters["Lift"].append(lift)
        print(f"Lift: {lift}")

    print("\n\nMaximum Lift Parameters")
    max_lift_index = parameters["Lift"].index(max(parameters["Lift"]))
    for param in parameters:
        print(f"{param}: {round(parameters[param][max_lift_index], 2)} {units[param]}")

    print("\nLowest Speed Parameters")
    min_speed_index = parameters["Speed"].index(min(parameters["Speed"]))
    for param in parameters:
        print(f"{param}: {round(parameters[param][min_speed_index], 2)} {units[param]}")

    plt.plot(parameters["Alpha"], parameters["Speed"])
    plt.plot(parameters["Alpha"], parameters["Lift"])
    plt.plot(parameters["Alpha"], parameters["Cl/Cd"])
    plt.legend(["Speed (m/s)", "Lift (N)", "Cl/Cd"])
    plt.show()
