import pexpect
import os
import sys
import time
import numpy as np
import re

# DOES NOT WORK ON WINDOWS, PLS USE LINUX/WSL/Cygwin
# Some installation notes for WSL2:
# Xfoil will not work without stuff below, because it uses special characters for graphs
# https://superuser.com/questions/1729488/wsl2-install-fonts-for-linux-gui-app-using-x-server
# Need answer by trpt4him to install venv on WSL because that's not included for some reason
# https://stackoverflow.com/questions/61528500/installing-venv-for-python3-in-wsl-ubuntu
# To install xfoil, do sudo apt-get install xfoil

def send_foil_command(com):
    xfoil.sendline(com)
    xfoil.expect("XFOIL   c>")

def send_foil_operation(com):
    xfoil.sendline(com)
    xfoil.expect(".OPERi   c>")

DAT_FILE = "fx63-137.dat"
DAT_FILE_PATH = f"./dat_files/{DAT_FILE}"
AIRFOIL = DAT_FILE.removesuffix(".dat").capitalize()

POLAR_DIR = f"./polars/{AIRFOIL}auto"
if not os.path.exists(POLAR_DIR):
    os.mkdir(POLAR_DIR)

convergence_re_patterns = [b"VISCAL:  Convergence failed", b".OPERva   c>"]
compiled_re_patters = [re.compile(pattern) for pattern in convergence_re_patterns]

reynolds_numbers = np.arange(30000, 150000, 10000)
for re in reynolds_numbers:
    xfoil = pexpect.spawn("xfoil")
    #xfoil.logfile = sys.stdout.buffer
    xfoil.expect("XFOIL   c>")
    send_foil_command(f"LOAD {DAT_FILE_PATH}")
    xfoil.sendline("MDES")
    xfoil.expect(".MDES   c>")
    xfoil.sendline("FILT")
    xfoil.expect(".MDES   c>")
    xfoil.sendline("EXEC")
    xfoil.expect(".MDES   c>")
    send_foil_command("")
    send_foil_command("PANE")

    # To disable GUI
    xfoil.sendline("PLOP")
    xfoil.expect(" Option, Value")
    xfoil.sendline("G")
    xfoil.expect(" Option, Value")
    send_foil_command("")

    send_foil_operation("OPER")
    send_foil_operation("ITER 2000")
    send_foil_operation(f"RE {re}")
    xfoil.sendline(f"VISC {re}")
    xfoil.expect(".OPERv   c>")
    xfoil.sendline("PACC")
    xfoil.expect("Enter  polar save")
    xfoil.sendline(f"{POLAR_DIR}/T1_Re{re}_M0_N9.txt")
    xfoil.expect("Enter  polar dump")
    xfoil.sendline("")
    xfoil.expect(".OPERva   c>")
    AOA_min = 0
    AOA_max = 20
    interval = 0.05
    aoas = np.arange(AOA_min, AOA_max, interval)
    #xfoil.sendline("ASeq 0 20 0.05")
    #xfoil.expect(".OPERva   c>", timeout=1000)
    for aoa in aoas:
        aoa = round(aoa, 3)
        converge = 0
        fail_cnt = 0
        while converge == 0:
            print(f"Running {aoa}")
            xfoil.sendline(f"ALFA {aoa}")
            converge = xfoil.expect_list(compiled_re_patters, timeout=100)
            if converge == 0:
                fail_cnt += 1
                print(f"FAILED {fail_cnt} times AT {aoa}")
            
            if fail_cnt > 4:
                # Give up
                print(f"CONVERGENCE FAILED 4x AT {aoa}")
                xfoil.sendline("INIT")
                xfoil.expect(".OPERva   c>")
                break
    xfoil.sendline("PACC")
    xfoil.expect(".OPERv   c>")
    xfoil.sendline("VISC")
    send_foil_command("")
    xfoil.sendline("QUIT")