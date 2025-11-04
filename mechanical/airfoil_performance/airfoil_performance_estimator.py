import math
import numpy as np
import os

CHAR_LEN = 0.05 # coord len in meters
WING_LEN = 0.25 #*2 # 0.25m estimate wing len that fits in the container - *2 for folding design 
MASS = 0.3

G = 9.81
MU_AIR = 1.46e-5
RHO_AIR = 1.225
WING_AREA = WING_LEN * CHAR_LEN
DRAG_FACTOR = 1
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
    re = reynolds_number(CHAR_LEN, v_prev)
    closest_re = min(airfoil_table, key=lambda x:abs(x-re))
    closest_alpha = min(airfoil_table[closest_re]["columns"]["alpha"], key=lambda x: abs(x-alpha))
    alpha_index = airfoil_table[closest_re]["columns"]["alpha"].index(closest_alpha)
    while abs(v-v_prev) > 0.3:
        v_prev = v
        re = reynolds_number(CHAR_LEN, v_prev)
        closest_re = min(airfoil_table, key=lambda x:abs(x-re))

        v = math.cbrt((2*E_descent)/(DRAG_FACTOR * airfoil_table[closest_re]["columns"]["CD"][alpha_index] * air_density * wing_area))
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
    alphas = np.linspace(-5, 18, 100)
    airfoil_table = {}
    for polar in os.listdir(".\\polars"):
        parsed_polar = parse_xflr5_polar(os.path.join(".\\polars", polar))
        airfoil_table[parsed_polar["metadata"]["reynolds"]] = parsed_polar

    lifts = []
    for alpha in alphas:
        v = speed_solver(airfoil_table, alpha, WING_AREA, MASS, RHO_AIR)
        print(reynolds_number(CHAR_LEN, v))
        cl, cd = obtain_cl_cd(airfoil_table, v, alpha)
        v_horizontal = math.sqrt(v**2 - v_descent**2)
        print(f"Alpha (wing angle): {alpha}, Speed (diagonally x,y): {v}, Speed (x, horizontal): {v_horizontal}, Cl: {cl}, Cd: {cd}")
        lift = get_lift_data(cl, WING_AREA, RHO_AIR, v)
        lifts.append(lift)
        print(f"Lift: {lift}")

    print(f"Highest Lift: {round(max(lifts), 3)}N")

