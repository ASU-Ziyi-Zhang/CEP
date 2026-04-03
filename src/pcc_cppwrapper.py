################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import os
from ctypes import *

### See also
### https://stackoverflow.com/questions/24640817/python-ctypes-definition-for-c-struct
### https://stackoverflow.com/questions/62277376/python-ctypes-pointer-to-struct-as-identifier-without-member-access
### https://tristan-ka.github.io/IBOAT_RL/_downloads/SIMULINK_TO_C__PYTHON.pdf

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

class controller(c_void_p):
    # subclassing c_void_p creates an opaque pointer type that is distinct
    # from c_void_p, and can only be instantiated as a pointer
    # We do not care to provide hints on the controller struct fields
    pass

class inputs(Structure):
    ### From the <model_name>.h autogen file
    # /* External inputs (root inport signals with default storage) */
    # struct ExtU_longitudinal_mpc_T {
    #   real_T t;                            /* '<Root>/t' */
    #   real_T ego_state[3];                 /* '<Root>/ego_state' */
    #   real_T pv_state[3];                  /* '<Root>/pv_state' */
    #   real_T pos_pred[201];                /* '<Root>/pos_pred' */
    #   real_T vel_pred[201];                /* '<Root>/vel_pred' */
    #   real_T acc_pred[201];                /* '<Root>/acc_pred' */
    #   real_T time_pred[201];               /* '<Root>/time_pred' */
    #   real_T param_vec[21];                /* '<Root>/param_vec' */
    # };
    '''Create a ctypes struct to match ExtU_<model_name>_T'''
    _fields_ = [
        ('t', c_double),
        ('ego_state', c_double * 3),
        ('pv_state', c_double * 3),
        ('pos_pred', c_double * 201),
        ('vel_pred', c_double * 201),
        ('acc_pred', c_double * 201),
        ('time_pred', c_double * 201),
        ('param_vec', c_double * 21)
    ]

class outputs(Structure):
    ### From the longitudinal_mpc0.h autogen file
    # struct ExtY_longitudinal_mpc_T {
    #   real_T acc_des;                      /* '<Root>/acc_des' */
    #   real_T state_trajectory[128];        /* '<Root>/state_trajectory' */
    #   real_T control_trajectory[32];       /* '<Root>/control_trajectory' */
    #   real_T time_trajectory[32];          /* '<Root>/time_trajectory' */
    #   real_T slacks[5];                    /* '<Root>/slacks' */
    #   real_T reference[4];                 /* '<Root>/reference' */
    #   real_T constraint;                   /* '<Root>/constraint' */
    #   real_T cost;                         /* '<Root>/cost' */
    #   flags exitflag;                      /* '<Root>/exitflag' */
    #   real_T iters;                        /* '<Root>/iters' */
    # };

    '''Create a ctypes struct to match ExtY_<model_name>_T'''
    _fields_ = [
        ('acc_des', c_double),
        ('state_trajectory', c_double * (4*32)),
        ('control_trajectory', c_double * 32),
        ('time_trajectory', c_double * 32),
        ('slacks', c_double * 5),
        ('reference', c_double * 4),
        ('constraint', c_double),
        ('cost', c_double),
        ('exitflag', c_int), # flags is a custom enum structure - will just cast to an int "unsafely"
        ('iters', c_double)
    ]

class cpp_api(object):
    '''
        API Wrapper of autogen C++ Code from Matlab/Simulink coder
        
        Expected call order of methods is:
            set fields of inputs_p.contents
            
            step_inputs() -> step_controller() -> step_outputs()
            
            read fields of outputs_p.contents

        When done with the shared library, call the cleanup method:
            cleanup()
        
        Optionally accepts either the absolute path to 'libraryname' or checks for 'libraryname' in the current path
    '''
    ### From the pccmpc_api.h custom file
    # // Typedefs of the name of the autogen model for convenience - Change these based on Simulink Model name
    # typedef <model_name> AUTOGENCLASS;
    # typedef ExtU_<model_name>_T AUTOGENINPUT;
    # typedef ExtY_<model_name>_T AUTOGENOUTPUT;

    # // External API bindings
    # extern "C" {
    #     // Object constructors
    #     AUTOGENCLASS* new_controller();
    #     AUTOGENINPUT* new_inputs();
    #     AUTOGENOUTPUT* new_outputs();

    #     // Object methods
    #     void init_controller(AUTOGENCLASS* controller);

    #     void set_inputs(AUTOGENCLASS* controller, AUTOGENINPUT* inputs);
    #     void step_controller(AUTOGENCLASS* controller);
    #     void get_outputs(AUTOGENCLASS* controller, AUTOGENOUTPUT* outputs);
        
    #     // Object destructors
    #     void cleanup_controller(AUTOGENCLASS* controller);
    #     void cleanup_inputs(AUTOGENCLASS* inputs);
    #     void cleanup_outputs(AUTOGENCLASS* outputs);
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
                filename = 'pcc_so'
                fileext = '.dll'

            elif os.name == 'posix': # Linux
                filename = 'libpcc_so'
                fileext = '.so'

            elif os.name == 'darwin': # Mac
                filename = 'libpcc_so'
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
        self.lib.new_controller.restype = controller # result type of new_controller c function is a pointer, but we do not care as to what on Python side
        self.lib.new_inputs.restype = POINTER(inputs) # result type of new_inputs c function should be a pointer of python.ctypes inputs class
        self.lib.new_outputs.restype = POINTER(outputs)

        self.lib.init_controller.argtypes = [controller]

        self.lib.set_inputs.argtypes = [controller, POINTER(inputs)]
        self.lib.step_controller.argtypes = [controller]
        self.lib.get_outputs.argtypes = [controller, POINTER(outputs)]

        self.lib.cleanup_controller.argtypes = [controller]
        self.lib.cleanup_inputs.argtypes = [POINTER(inputs)]
        self.lib.cleanup_outputs.argtypes = [POINTER(outputs)]

        # Create object pointer for controller
        self.controller_p = self.lib.new_controller()
        
        # Create object pointers for inputs and outputs - since we want to work with the struct data indicate to ctypes the fields
        self.inputs_p = self.lib.new_inputs()
        self.outputs_p = self.lib.new_outputs()

        ### Initialize
        self.initialize()

        # Print success status
        print(f'C++ Wrapper constructed for {file}')

    def step_inputs(self):
        '''Readies the controller inputs struct for use in the control step'''
        # Set controller inputs
        self.lib.set_inputs(self.controller_p, self.inputs_p)

    def step_controller(self):
        '''Steps the controller'''
        # Step controller
        self.lib.step_controller(self.controller_p)

    def step_outputs(self):
        '''Readies the controller outputs struct to get new results'''
        # Get controller outputs
        self.lib.get_outputs(self.controller_p, self.outputs_p)

    def initialize(self):
        '''Runs the initialization routine for the model'''
        self.lib.init_controller(self.controller_p)

    def cleanup(self):
        '''Frees memory allocated by control model shared object and terminates model dependencies'''
        # Frees dynamically allocated memory
        self.lib.cleanup_controller(self.controller_p)
        self.lib.cleanup_inputs(self.inputs_p)
        self.lib.cleanup_outputs(self.outputs_p)

    def get_output(self, fieldname: chr):
        '''
            Returns 'fieldname' from the underlying output struct data in API

            Equivalent to the command r = api.outputs_p.contents.<fieldname>
        '''
        
        f = getattr(self.outputs_p.contents, fieldname)

        return f
    
    def set_input(self, v, fieldname: chr):
        '''
            Sets 'fieldname' in the underlying input struct data in API

            Equivalent to the command api.inputs_p.contents.<fieldname> = v
        '''
        
        setattr(self.inputs_p.contents, fieldname, v)