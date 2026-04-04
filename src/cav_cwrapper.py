################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import os
import time
from ctypes import *

### See also
### https://stackoverflow.com/questions/24640817/python-ctypes-definition-for-c-struct
### https://stackoverflow.com/questions/62277376/python-ctypes-pointer-to-struct-as-identifier-without-member-access
### https://tristan-ka.github.io/IBOAT_RL/_downloads/SIMULINK_TO_C__PYTHON.pdf

# nm -D libcav_so.so # Check the exported function names in the shared object
# Some Matlab functions aren't compiled correctly with C
# #ifdef __cplusplus export "C" {} #endif

def find_library_by_partial_name(search_dir, partial_name):
    """
    Searches for shared libraries in a given directory that contain a specified partial name.

    Parameters:
    search_dir (str): The directory where the search should be conducted.
    partial_name (str): The substring to look for in library filenames.

    Returns:
    list: A list of file paths matching the criteria.
    """
    matches = []  # Initialize an empty list to store matching file paths
    
    # Walk through the directory structure
    for root, _, files in os.walk(search_dir):
        for file in files:
            # Check if the filename contains the partial string and ends with a shared library extension
            if partial_name in file and (file.endswith(".so") or file.endswith(".dll")):
                matches.append(os.path.join(root, file))  # Store the full path of the matching file
    
    return matches  # Return the list of matching library file paths

class c_api(object):
    '''
        API Wrapper of autogen C Code from Matlab/Simulink coder
        
        Expected call order of methods is:
            step_controller()

        When done with the shared library, call the cleanup method:
            cleanup()
        
        Optionally accepts either the absolute path to 'libraryname' or checks for 'libraryname' in the current path
    '''
    ### From the <model_name>.h custom file
    # // External API bindings
    # extern "C" {
    #    // methods
    #    <model_name>()
    #    <model_name>_terminate()

    # void CAV_ctrl_mdl_wTraJ_241004(
        # const double IntscInfo[30], const double SpdLimInfo[4],
        # const double CtrlInfo[14], const double CtrlPar[15],
        # const double StopInfo[2], double *aRef, double *vRef, double *sRef,
        # double *UpdType, double time_LTtraj[1000], double acc_LTtraj[1000],
        # double vel_LTtraj[1000], double pos_LTtraj[1000], double time_STtraj[100],
        # double acc_STtraj[100], double vel_STtraj[100], double pos_STtraj[100]);
    # }

    def __init__(self, libraryname=''):
        ### Initialize name of shared object
        # Check file extension of libraryname if provided
        # Split the extension from the path and normalise it to lowercase.
        ext = os.path.splitext(libraryname)[-1].lower()

        # Get full file path
        if libraryname == '': # If no filename provided in construction
            # Assume api_so is the shared library name with .dll or .so extension
            if os.name == 'nt': # Windows
                filename = 'cav_so'
                fileext = '.dll'
            
            elif os.name == 'posix': # Linux
                filename = 'libcav_so'
                fileext = '.so'

            elif os.name == 'darwin': # Mac
                filename = 'libcav_so'
                fileext = '.dylib'
            
            else:
                raise ValueError('cwrapper class cannot determine system type!')
            
            file = filename+fileext

        elif ext == '': # Filename provided but no extension
            if os.name == 'nt':
                fileext = '.dll'
            
            elif os.name == 'posix':
                fileext = '.so'

            elif os.name == 'darwin':
                fileext = '.dylib'
            
            else:
                raise ValueError('cwrapper class cannot determine system type!')
            
            file = libraryname+fileext

        else:
            file = libraryname
            
        # Load shared object
        if os.path.isfile(file):
            self.lib = cdll.LoadLibrary( os.path.join(os.getcwd(), file) )
        
        elif os.path.isdir(file):
            self.lib = cdll.LoadLibrary(file)

        else:
            raise ValueError(f'cwrapper class cannot find file {file}!')

        ### Provide hints to ctypes on the function return types and argument types
        self.lib.CAV_ctrl_mdl_wTraJ_241219.argtypes = [
            POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), 
            POINTER(c_double), POINTER(c_double), POINTER(c_double),
            POINTER(c_double),
            (c_double * 1000), (c_double * 1000), (c_double * 1000), (c_double * 1000),
            (c_double * 100), (c_double * 100), (c_double * 100), (c_double * 100)
        ]
        self.lib.CAV_ctrl_mdl_wTraJ_241219.restype = None # void in C/C++ means None, void pointer is a pointer that points to nothing

        # Initialize
        self.initialize()
        
        # Print success status
        print(f'C Wrapper constructed for {file}')

    def step_controller(self, IntscInfo, SpdLimInfo, CtrlInfo, CtrlPar, StopInfo):
        '''Steps the controller'''
        # Step controller
        # const double IntscInfo[30], 
        # const double SpdLimInfo[4],
        # const double CtrlInfo[14], 
        # const double CtrlPar[15],
        # const double StopInfo[2],

        # double *aRef, 
        # double *vRef,
        # double *sRef,
        # double *UpdType, 
        # double time_LTtraj[1000], double acc_LTtraj[1000], double vel_LTtraj[1000], double pos_LTtraj[1000], 
        # double time_STtraj[100], double acc_STtraj[100], double vel_STtraj[100], double pos_STtraj[100]

        ### Helper functions
        def assert_len(v, l):
            len_v = len(v)
            assert len_v == l, 'Python input array mis-sized! Was expecting len={:d} but got {:d}'.format(l, len_v)

        def make_c_var():
            # Make c variable
            v = c_double(0.)

            return v
        
        def make_c_pointer():
            # Make c pointer
            v = c_double(0.)
            p = pointer(v)

            return p
        
        def make_empty_c_array(l):
            # Make c array
            c_array = (c_double * l)()
            
            for i in range(0, l):
                c_array[i] = 0.

            return c_array

        def make_c_array(v, l):
            # Check size of input vector
            assert_len(v, l)

            # Make c array
            c_array = (c_double * l)()
            
            for i in range(0, l):
                c_array[i] = v[i]

            return c_array
        
        ### Generate c arrays from inputs and check size
        c_IntscInfo = make_c_array(IntscInfo, 30)
        c_SpdLimInfo = make_c_array(SpdLimInfo, 4)
        c_CtrlInfo = make_c_array(CtrlInfo, 14)
        c_CtrlPar = make_c_array(CtrlPar, 15)
        c_StopInfo = make_c_array(StopInfo, 2)

        # Make variables that will get the outputs from c library method
        aRef = make_c_var()
        vRef = make_c_var()
        sRef = make_c_var()

        UpdType = make_c_var()

        time_LTtraj = make_empty_c_array(1000)
        acc_LTtraj = make_empty_c_array(1000)
        vel_LTtraj = make_empty_c_array(1000)
        pos_LTtraj = make_empty_c_array(1000)

        time_STtraj = make_empty_c_array(100)
        acc_STtraj = make_empty_c_array(100)
        vel_STtraj = make_empty_c_array(100)
        pos_STtraj = make_empty_c_array(100)

        ### Step the control model - the output arguments are stored in the pointers aRef, vRef...
        self.lib.CAV_ctrl_mdl_wTraJ_241219(
            c_IntscInfo, c_SpdLimInfo, c_CtrlInfo, c_CtrlPar, c_StopInfo,
            byref(aRef), byref(vRef), byref(sRef),
            byref(UpdType),
            time_LTtraj, acc_LTtraj, vel_LTtraj, pos_LTtraj,
            time_STtraj, acc_STtraj, vel_STtraj, pos_STtraj
        )

        # Outputs
        return aRef.value, vRef.value, sRef.value, UpdType.value, time_LTtraj[:], acc_LTtraj[:], vel_LTtraj[:], pos_LTtraj[:], time_STtraj[:], acc_STtraj[:], vel_STtraj[:], pos_STtraj[:]

    def initialize(self):
        '''Runs the initialization routine for the model'''
        self.lib.CAV_ctrl_mdl_wTraJ_241219_initialize()
    
    def cleanup(self):
        '''Frees memory allocated by control model shared object and terminates model dependencies'''
        # Frees dynamically allocated memory and loaded libraries
        self.lib.CAV_ctrl_mdl_wTraJ_241219_terminate()