import pexpect
import os
import sys
import time

# DOES NOT WORK ON WINDOWS, PLS USE LINUX/WSL/Cygwin

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