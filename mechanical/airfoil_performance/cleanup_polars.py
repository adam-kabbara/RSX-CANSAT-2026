import os

if __name__ == "__main__":
    for polar_dir in os.listdir("./polars"):
        dir_path = f"./polars/{polar_dir}"
        if len(os.listdir(dir_path)) == 0:
            print(f"Removing empty directory: {dir_path}")
            os.rmdir(dir_path)
        else:
            for polar_file in os.listdir(dir_path):
                file_path = f"{dir_path}/{polar_file}"
                short = False
                with open(file_path, "r") as f:
                    lines = f.readlines()
                    if len(lines) < 15:
                        print(f"Removing incomplete polar file: {file_path}")
                        short = True
                if short:
                    os.remove(file_path)