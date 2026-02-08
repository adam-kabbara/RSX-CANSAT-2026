#!/usr/bin/env python3
import subprocess
import sys
import argparse
import re


def get_device_id(device_name=None):
    try:
        output = subprocess.check_output(['xinput', 'list'], text=True)
    except FileNotFoundError:
        print("Error: 'xinput' is not installed.")
        sys.exit(1)
    if device_name:
        for line in output.splitlines():
            if device_name.lower() in line.lower() and 'id=' in line:
                match = re.search(r'id=(\d+)', line)
                if match:
                    return match.group(1)
        print(f"Error: Device '{device_name}' not found.")
        sys.exit(1)
    else:
        for line in output.splitlines():
            if re.search(r'(touchpad|trackpad)', line, re.IGNORECASE) and 'pointer' in line:
                match = re.search(r'id=(\d+)', line)
                if match:
                    return match.group(1)
        print("Error: No touchpad detected.")
        print("Run 'xinput list' manually to find the name and pass it using -d or --device.")
        sys.exit(1)


def set_orientation(device_id, orientation):
    matrices = {
        'normal': "1 0 0 0 1 0 0 0 1",
        'inverted': "-1 0 1 0 -1 1 0 0 1",
        'left': "0 1 0 -1 0 1 0 0 1",  # counter-clockwise
        'right': "0 -1 1 1 0 0 0 0 1"  # clockwise
    }

    if orientation not in matrices:
        print(f"Invalid orientation: {orientation}")
        sys.exit(1)

    matrix = matrices[orientation]
    cmd = [
              'xinput', 'set-prop',
              device_id,
              '--type=float',
              'Coordinate Transformation Matrix'
          ] + matrix.split()

    print(f"Setting device ID {device_id} to '{orientation}'...")
    try:
        subprocess.run(cmd, check=True)
        print("Success.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing xinput: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Change touchpad orientation on X11.")
    parser.add_argument(
        '--orientation', "-o",
        default='left',
        choices=['left', 'right', 'normal', 'inverted'],
        help="The direction of the touchpad. Left (counter-clockwise) is the default."
    )
    parser.add_argument(
        '--device', '-d',
        type=str,
        help="Specific device name (substring match) if auto-detection fails."
    )

    args = parser.parse_args()

    device_id = get_device_id(args.device)
    set_orientation(device_id, args.orientation)


if __name__ == "__main__":
    main()