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
    v_prev = 0
    v = v_init
    while abs(v-v_prev) > 0.3:
        v_prev = v
        re = reynolds_number(CHAR_LEN, v)
        closest_re = min(airfoil_table, key=lambda x:abs(x-re))
        closest_alpha = min(airfoil_table[closest_re]["columns"]["alpha"], key=lambda x: abs(x-alpha))
        alpha_index = airfoil_table[closest_re]["columns"]["alpha"].index(closest_alpha)
        v = math.cbrt((2*E_descent)/(airfoil_table[closest_re]["columns"]["CD"][alpha_index] * air_density * wing_area))
        print(abs(v-v_prev))
    return v

def lift_optimizer(airfoil_table, wing_area, mass, air_density, v_init=12):
    E_descent = mass * G * v_descent
    # E_descent = 0.5 * air_density * wing_area * airfoil_table[re]["columns"]["CD"][some_index] * v**3
    lift_margin = 1 # N
    min_lift = mass * G + lift_margin

    v_prev = 0
    v = v_init
    cl_index = 0
    re = 0
    closest_re = 0

    tol = 0.1
    while abs(v-v_prev) > tol:
        v_prev = v
        re = reynolds_number(CHAR_LEN, v_prev)
        closest_re = min(airfoil_table, key=lambda x:abs(x-re))
        best_cl = max(airfoil_table[closest_re]["columns"]["CL"])
        cl_index = airfoil_table[closest_re]["columns"]["CL"].index(best_cl)

        v = math.sqrt((2*min_lift)/(air_density*wing_area*airfoil_table[closest_re]["columns"]["CL"][cl_index]))
    
    # Sanity check
    if 0.5 * air_density * wing_area * airfoil_table[closest_re]["columns"]["CD"][cl_index] * v**3 > E_descent:
        print("Too fast for given energy")
    

    best_alpha = airfoil_table[closest_re]["columns"]["alpha"][cl_index]
    return v, best_alpha


def get_lift_data(cl, wing_area, air_density, speed):
    return cl * air_density * ((speed**2)/2) * wing_area


def get_ld_coef(force, wing_area, air_density, speed):
    return (2 * force)/(air_density * speed **2 * wing_area)

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


def run_speed_solver(a_min, a_max, airfoil=AIRFOIL_NAME):
    alphas = np.linspace(0, 20, 100)
    airfoil_table = {}
    for polar in os.listdir(f".\\polars\\{airfoil}"):
        parsed_polar = parse_xflr5_polar(os.path.join(f".\\polars\\{airfoil}", polar))
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


def run_min_lift_solver(airfoil):
    airfoil_table = {}
    for polar in os.listdir(f".\\polars\\{airfoil}"):
        parsed_polar = parse_xflr5_polar(os.path.join(f".\\polars\\{airfoil}", polar))
        airfoil_table[parsed_polar["metadata"]["reynolds"]] = parsed_polar

    v, alpha = lift_optimizer(airfoil_table, WING_AREA, MASS, RHO_AIR)
    parameters = {"Alpha" : [], "Speed": [], "Re": [], "Cl": [], "Cd": [], "Cl/Cd": [], "Lift": [], "Drag": []}
    units = {"Alpha" : "deg", "Speed": "m/s", "Re": "", "Cl": "", "Cd": "", "Cl/Cd": "", "Lift": "N", "Drag": "N"}
    cl, cd = obtain_cl_cd(airfoil_table, v, alpha)
    v_horizontal = math.sqrt(v**2 - v_descent**2)
    lift = get_lift_data(cl, WING_AREA, RHO_AIR, v)
    drag = get_lift_data(cd, WING_AREA, RHO_AIR, v)

    parameters["Alpha"].append(alpha)
    parameters["Speed"].append(v)
    parameters["Re"].append(reynolds_number(CHAR_LEN, v))
    parameters["Cl"].append(cl)
    parameters["Cd"].append(cd)
    parameters["Cl/Cd"].append(cl/cd)
    parameters["Lift"].append(lift)
    parameters["Drag"].append(drag)

    for param in parameters:
        print(f"{param}: {round(parameters[param][0], 2)} {units[param]}")

    print(f"Max drag: {round(MASS * G, 2)}N --> Available Drag: {round(MASS * G - drag, 2)}N")
    print(f"Ground Speed: {round(v_horizontal, 2)}m/s")

    return parameters, units

def compare_airfoils():
    airfoils = [path for path in os.listdir(f".\\polars")]    

    units = None

    lowest_speed = math.inf
    best_lift_params = None
    best_lift_airfoil = None

    lowest_drag = math.inf
    lowest_drag_params = None
    lowest_drag_airfoil = None

    airfoil_params = {}
    for airfoil in airfoils:
        parameters, units = run_min_lift_solver(airfoil)
        airfoil_params[airfoil] = parameters
        if parameters["Speed"][0] < lowest_speed:
            lowest_speed = parameters["Speed"][0]
            best_lift_params = parameters
            best_lift_airfoil = airfoil
        
        if parameters["Drag"][0] < lowest_drag:
            lowest_drag = parameters["Drag"][0]
            lowest_drag_params = parameters
            lowest_drag_airfoil = airfoil

    print(f"\n\nBEST AIRFOIL (highest lift @ slowest speed): {best_lift_airfoil}")
    for param in best_lift_params:
        print(f"{param}: {round(best_lift_params[param][0], 2)} {units[param]}")

    drag = best_lift_params["Drag"][0]
    print(f"Max usable drag: {round(MASS * G, 2)}N --> Available Drag: {round(MASS * G - drag, 2)}N")

    cd = best_lift_params["Cd"][0]
    max_cd = get_ld_coef(MASS * G, WING_AREA, RHO_AIR, best_lift_params["Speed"][0])
    print(f"Max allowable CD: {round(max_cd, 4)} --> Available CD: {round(max_cd-cd, 4)}")
    v_ground = math.sqrt(best_lift_params["Speed"][0]**2 - v_descent**2)
    print(f"Ground Speed: {round(v_ground, 2)}m/s --> {round(v_ground*3.6, 2)}km/h")

    print(f"\n\nBEST AIRFOIL (lowest drag @ slowest speed): {lowest_drag_airfoil}")
    for param in lowest_drag_params:
        print(f"{param}: {round(lowest_drag_params[param][0], 2)} {units[param]}")

    drag = lowest_drag_params["Drag"][0]
    print(f"Max usable drag: {round(MASS * G, 2)}N --> Available Drag: {round(MASS * G - drag, 2)}N")

    cd = lowest_drag_params["Cd"][0]
    max_cd = get_ld_coef(MASS * G, WING_AREA, RHO_AIR, lowest_drag_params["Speed"][0])
    print(f"Max allowable Cd: {round(max_cd, 4)} --> Available Cd: {round(max_cd-cd, 4)}")
    v_ground = math.sqrt(lowest_drag_params["Speed"][0]**2 - v_descent**2)
    print(f"Ground Speed: {round(v_ground, 2)}m/s --> {round(v_ground*3.6, 2)}km/h")


    speed = []
    cl = []
    cd = []
    for airfoil in airfoil_params:
        speed.append(airfoil_params[airfoil]["Speed"][0])
        cl.append(airfoil_params[airfoil]["Cl"][0])
        cd.append(airfoil_params[airfoil]["Cd"][0])

    plt.scatter(cd, cl)
    for i, txt in enumerate(airfoil_params):
        plt.annotate(txt, (cd[i], cl[i]), textcoords="offset points", xytext=(0,10), ha='center')

    plt.xlabel("Cd")
    plt.ylabel("Cl")
    plt.title("Airfoil Cl VS Cd @ minimum lift")
    plt.show()
    

if __name__ == "__main__":
    #run_speed_solver(0, 20, AIRFOIL_NAME)
    compare_airfoils()
    