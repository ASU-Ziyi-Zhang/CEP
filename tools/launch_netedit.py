#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import subprocess
import os
import sys

def launch_exe_from_pip_package(package_name, exe_name):
    '''
    Attempt to run exe that is located in a specific python package installation location
    
    Example usage:

        launch_exe_from_package('your_package_name', 'your_executable.exe')
    '''
    
    # Get the path to the package's installation folder
    package_path = next(p for p in sys.path if 'site-packages' in p)
    
    # Construct the full path to the executable
    exe_path = os.path.join(package_path, package_name, exe_name)
    print(exe_path)
    # Launch the executable
    try:
        subprocess.run([exe_path], check=True)

    except subprocess.CalledProcessError as e:
        print(f"Failed to launch {exe_name}: {e}")

def launch_exe_from_package(package_name, exe_name):
    '''
    Attempt to run exe that is located in a specific python package installation location
    
    Example usage:

        launch_exe_from_package('your_package_name', 'your_executable.exe')
    '''
    
    # Get the path to the package's installation folder
    package_path = os.path.dirname(sys.modules[package_name].__file__)
    
    # Construct the full path to the executable
    exe_path = os.path.join(package_path, exe_name)
    
    # Launch the executable
    try:
        subprocess.run([exe_path], check=True)

    except subprocess.CalledProcessError as e:
        print(f"Failed to launch {exe_name}: {e}")

if __name__ == '__main__':
    launch_exe_from_pip_package( 'sumo', os.path.join('bin', 'netedit.exe') )

    print('Done.')