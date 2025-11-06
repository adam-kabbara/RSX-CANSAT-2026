def get_coords(dat_file: str) -> tuple[str, list]:
    with open(dat_file, "r") as f:
        lines = f.readlines()
        name = lines[0].strip("\n")
        coords = []
        for line in lines[1:]:
            str_coords = line.split()
            x, y = float(str_coords[0]), float(str_coords[1])
            coords.append([x, y])
        return name, coords
    

def transform_coords(chord_depth: float, unitless_coords: list) -> list:
    mm_coords = []
    for coord in unitless_coords:
        x_mm, y_mm = coord[0] * chord_depth, coord[1] * chord_depth
        mm_coords.append([x_mm, y_mm])
    return mm_coords


def write_coords(name: str, chord_depth: float, mm_coords: list) -> None:
    with open(f"./sldwork_points/{name}_{round(chord_depth, 2)}.txt", "w") as f:
        for coord in mm_coords:
            f.write(f"{coord[0]} {coord[1]} {0}\n")
        

if __name__ == "__main__":
    CHORD_DEPTH = 50 # in mm
    name, unitless_coords = get_coords("./dat_files/s1223.dat")
    mm_coords = transform_coords(CHORD_DEPTH, unitless_coords)
    write_coords(name, CHORD_DEPTH, mm_coords)
