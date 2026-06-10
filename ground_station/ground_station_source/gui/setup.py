from cx_Freeze import setup, Executable

# Dependencies are automatically detected, but it might need
# fine tuning.
# Run command: python setup.py bdist_msi
build_options = {'packages': [], 'excludes': [], 'include_files': ["icon.png", "cansat_2023_simp.txt"]}

bdist_msi_options = {
    'upgrade_code': '{77d998f8-74c5-41f1-a150-929695313ea0}',
    'add_to_path': False,
    'initial_target_dir': r"[DesktopFolder]\RSX\CANSAT",
}

base = 'gui'

executables = [
    Executable('rsx_cansat_gui.py', base=base, target_name = 'RSX-CansatGUI', icon="icon.ico")
]

setup(name='RSX CANSAT GUI',
      version = '1',
      author="RSX",
      description = 'CANSAT GUI',
      options = {'bdist_msi': bdist_msi_options, 'build_exe': build_options},
      executables = executables)
