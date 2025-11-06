import pexpect
import os
import sys
import time

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

DAT_FILE = "mh32.dat"
DAT_FILE_PATH = f"./dat_files/{DAT_FILE}"

xfoil = pexpect.spawn("xfoil")
xfoil.logfile = sys.stdout.buffer

xfoil.expect("XFOIL   c>")
send_foil_command(f"LOAD {DAT_FILE_PATH}")
xfoil.sendline("MDES")
xfoil.expect(".MDES   c>")
xfoil.sendline("FILT")
xfoil.expect(".MDES   c>")
xfoil.sendline("EXEC")
xfoil.expect(".MDES   c>")
xfoil.sendline("")
xfoil.expect("XFOIL   c>")
send_foil_command("PANE")
send_foil_operation("OPER")
send_foil_operation("ITER 200")
send_foil_operation("RE 50000")
xfoil.sendline("VISC 50000")
xfoil.expect(".OPERv   c>")
xfoil.sendline("PACC")
xfoil.expect("Enter  polar save")
xfoil.sendline("test.txt")
xfoil.expect("Enter  polar dump")
xfoil.sendline("")
xfoil.expect(".OPERva   c>")
xfoil.sendline("ALFA 0")
xfoil.expect(".OPERva   c>")
xfoil.sendline("PACC")
xfoil.expect(".OPERv   c>")
xfoil.sendline("VISC")
send_foil_command("")
xfoil.sendline("QUIT")