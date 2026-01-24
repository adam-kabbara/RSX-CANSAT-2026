import math
import numpy as np
import os
import matplotlib.pyplot as plt

CHAR_LEN = 0.36 # coord len in meters
WING_LEN = 0.08 #*2 # 0.25m estimate wing len that fits in the container - *2 for folding design 

G = 9.81
MU_AIR = 1.46e-5 # Kinematic viscosity of air @20C
RHO_AIR = 1.225 # Density of air @20C
WING_AREA = WING_LEN * CHAR_LEN # Assuming constant chord width (if we want to do variable we have to recompute Re as we move down the wing)

def reynolds_number(char_len, speed):
    re = char_len*speed/MU_AIR
    return re

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


if __name__ == "__main__":
    reynolds_numbers = [275000, 300000, 325000, 350000, 375000, 400000, 450000, 475000, 500000, 525000]
    for re in reynolds_numbers:
        speed = re * MU_AIR / CHAR_LEN

        ld_integrals = {}
        for airfoil_dir in os.listdir("./polars"):
            dir_path = f"./polars/{airfoil_dir}"
            airfoil_name = airfoil_dir.removesuffix("-ilauto")
            file_path = None
            for polar_file in os.listdir(dir_path):
                if f"Re{re}" in polar_file:
                    file_path = f"{dir_path}/{polar_file}"
                    break
            if not file_path:
                continue

            data = parse_xflr5_polar(file_path)
            cl_values = data["columns"]["CL"]
            cd_values = data["columns"]["CD"]
            ld_values = []
            ld_integrals[airfoil_name] = 0
            for cl, cd in zip(cl_values, cd_values):
                ld_values.append(cl/cd if cd != 0 else float('inf'))
                ld_integrals[airfoil_name] += (cl/cd if cd != 0 else 0)

        top_10_airfoils = sorted(ld_integrals.items(), key=lambda x: x[1], reverse=True)[:10]
        for airfoil_name, _ in top_10_airfoils:
            data = parse_xflr5_polar(f"./polars/{airfoil_name}-ilauto/T1_Re{re}_M0_N9.txt")
            cl_values = data["columns"]["CL"]
            cd_values = data["columns"]["CD"]
            ld_values = []
            lift_values = [get_lift_data(cl, WING_AREA, RHO_AIR, speed) for cl in cl_values]
            for cl, cd in zip(cl_values, cd_values):
                ld_values.append(cl/cd if cd != 0 else float('inf'))

            #plt.plot(data["columns"]["alpha"], ld_values, label=f"{airfoil_name}")
            plt.plot(data["columns"]["alpha"], lift_values, label=f"{airfoil_name}")
        plt.title(f"Lift (N) vs AoA at Re {re}")
        plt.xlabel("Angle of Attack (degrees)")
        plt.ylabel("Lift (N)")
        plt.legend([airfoil_name for airfoil_name, _ in top_10_airfoils])
        plt.show()
            