import pexpect
import os
import sys
import time
import numpy as np
import re as regex

# DOES NOT WORK ON WINDOWS, PLS USE LINUX/WSL/Cygwin
# Some installation notes for WSL2:
# Xfoil will not work without stuff below, because it uses special characters for graphs
# https://superuser.com/questions/1729488/wsl2-install-fonts-for-linux-gui-app-using-x-server
# Need answer by trpt4him to install venv on WSL because that's not included for some reason
# https://stackoverflow.com/questions/61528500/installing-venv-for-python3-in-wsl-ubuntu
# To install xfoil, do sudo apt-get install xfoil

def send_foil_command(xfoil, com):
    xfoil.sendline(com)
    xfoil.expect("XFOIL   c>")

def send_foil_operation(xfoil, com):
    xfoil.sendline(com)
    xfoil.expect(".OPERi   c>")

def run_xfoil(dat_file: str, re_min: int = 30000, re_max: int = 140000, re_step: int = 10000, aoa_min: int = 0, aoa_max: int = 20, aoa_step: float = 1):
    DAT_FILE_PATH = f"./scraped_dat/{dat_file}"
    AIRFOIL = dat_file.removesuffix(".dat")

    POLAR_DIR = f"./polars/{AIRFOIL}auto"
    if not os.path.exists(POLAR_DIR):
        os.mkdir(POLAR_DIR)

    if len(os.listdir(POLAR_DIR)) > 10:
        print(f"Polar directory {POLAR_DIR} already has files, skipping XFOIL run.")
        return

    convergence_re_patterns = [b"VISCAL:  Convergence failed", b".OPERva   c>"]
    compiled_re_patterns = [regex.compile(pattern) for pattern in convergence_re_patterns]

    bad_dat = [b"File READ error.  Unrecognizable file format", b"XFOIL   c>"]
    compiled_bad_dat = [regex.compile(pattern) for pattern in bad_dat]

    reynolds_numbers = np.arange(re_min, re_max, re_step)
    for re in reynolds_numbers:
        xfoil = pexpect.spawn("xfoil")
        xfoil.logfile = sys.stdout.buffer
        xfoil.expect("XFOIL   c>")
        xfoil.sendline(f"LOAD {DAT_FILE_PATH}")
        xfoil.expect_list(compiled_bad_dat)
        if xfoil.after == bad_dat[0]:
            print(f"BAD DAT FILE: {dat_file}")
            xfoil.sendline("QUIT")
            return
        
        xfoil.sendline("PPAR")
        xfoil.expect("Change what ?")
        xfoil.sendline("N 300")
        xfoil.expect("Change what ?")
        xfoil.sendline("T 1")
        xfoil.expect("Change what ?")
        xfoil.sendline("")  # exit PPAR
        xfoil.expect("Change what ?")
        xfoil.sendline("")  # exit PPAR
        xfoil.expect("XFOIL   c>")
        try:
            xfoil.sendline("MDES")
            xfoil.expect(".MDES   c>")
        except pexpect.exceptions.EOF:
            print(f"XFOIL CRASHED ON MDES FOR {dat_file} AT RE {re}")
            return
        xfoil.sendline("FILT")
        xfoil.expect(".MDES   c>")
        xfoil.sendline("EXEC")
        xfoil.expect(".MDES   c>")
        send_foil_command(xfoil, "")
        send_foil_command(xfoil, "PANE")

        # To disable GUI
        xfoil.sendline("PLOP")
        xfoil.expect(" Option, Value")
        xfoil.sendline("G")
        xfoil.expect(" Option, Value")
        send_foil_command(xfoil, "")

        send_foil_operation(xfoil, "OPER")
        send_foil_operation(xfoil, "ITER 1000")
        send_foil_operation(xfoil, f"RE {re}")
        xfoil.sendline(f"VISC {re}")
        xfoil.expect(".OPERv   c>")
        xfoil.sendline("PACC")
        xfoil.expect("Enter  polar save")
        xfoil.sendline(f"{POLAR_DIR}/T1_Re{re}_M0_N9.txt")
        xfoil.expect("Enter  polar dump")
        xfoil.sendline("")
        xfoil.expect(".OPERva   c>")
        aoas = np.arange(aoa_min, aoa_max, aoa_step)
        #xfoil.sendline("ASeq 0 20 0.05")
        #xfoil.expect(".OPERva   c>", timeout=1000)
        crash = False
        for aoa in aoas:
            if crash: break
            aoa = round(aoa, 3)
            converge = 0
            fail_cnt = 0
            try:
                while converge == 0:
                    print(f"Running {aoa}")
                    xfoil.sendline(f"ALFA {aoa}")
                    try:
                        xfoil.expect_list(compiled_re_patterns, timeout=100)
                    except pexpect.exceptions.EOF:
                        print(f"Crash at {aoa}, skip to next RE")
                        crash = True
                        break
                    if converge == 0:
                        fail_cnt += 1
                        print(f"FAILED {fail_cnt} times AT {aoa}")
                    
                    if fail_cnt > 4:
                        # Give up
                        print(f"CONVERGENCE FAILED 4x AT {aoa}")
                        xfoil.sendline("INIT")
                        xfoil.expect(".OPERva   c>", timeout=10)
                        break
            except pexpect.exceptions.TIMEOUT:
                print(f"TIMEOUT at {aoa}, skip to next RE")
                crash = True
                break
        if crash:
            continue
        try:
            xfoil.sendline("PACC")
            xfoil.expect(".OPERv   c>")
            xfoil.sendline("VISC")
            send_foil_command(xfoil, "")
            xfoil.sendline("QUIT")
        except:
            print(f"XFOIL CRASHED AT QUIT FOR {dat_file} AT RE {re}")
            continue


if __name__ == "__main__":
    dat_files = os.listdir("./scraped_dat")
    for dat_file in dat_files:
    #dat_file = "naca644421-il.dat"
        print(f"Running XFOIL for {dat_file}")
        run_xfoil(dat_file, re_min=275000, re_max=575000, re_step=25000, aoa_min=0, aoa_max=20, aoa_step=1)