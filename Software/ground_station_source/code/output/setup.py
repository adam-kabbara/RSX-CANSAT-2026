from cx_Freeze import setup, Executable
import os
import sys
# Dependencies are automatically detected, but it might need
# fine tuning.
# Run command: python setup.py bdist_msi

loc = os.path.dirname(os.path.abspath(__file__))
parent = os.path.abspath(os.path.join(loc, ".."))

sys.path.insert(0, parent)

include_files = [
    (os.path.join(parent, "media"), "media"),
    (os.path.join(parent, "command"), "command"),
    (os.path.join(parent, "data"), "data"),
    (os.path.join(parent, "gui"), "gui"),
    (os.path.join(parent, "plotter"), "plotter"),
    (os.path.join(parent, "serial"), "serial")
]

build_options = {'packages': [], 'excludes': [], 'include_files': include_files}

bdist_msi_options = {
    'upgrade_code': '{77d998f8-74c5-41f1-a150-929695313ea0}',
    'add_to_path': False,
    'initial_target_dir': r"[DesktopFolder]\RSX\CANSAT",
}

# Only for windows:
# base = 'gui'

executables = [
    Executable(script=os.path.join(parent, "main.py"), base=base, target_name = 'RSX-CansatGUI', icon=os.path.join(parent,"media/icon.ico"))
]

setup(name='RSX Aerial Command Center',
      version = '2',
      author="RSX",
      description = 'CANSAT GUI',
      options = {'bdist_msi': bdist_msi_options, 'build_exe': build_options},
      executables = executables)
