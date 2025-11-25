///////////////////////////////////////
/// Tyler Ard                       ///
/// Vehicle Mobility Systems Group  ///
/// tard(at)anl(dot)gov             ///
///////////////////////////////////////

#define S_FUNCTION_NAME  serialize_sim
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdint.h>
#include <string.h>

#include "serial_utils.h"

static void mdlInitializeSizes(SimStruct *S) {
    // Set function parameters
    ssSetNumSFcnParams(S, 0);

    // Define separate input ports for each field in the BSM structure
    // Define input port data types for each field in the BSM structure
    /* SS_DOUBLE (double-precision floating-point)
    SS_SINGLE (single-precision floating-point)
    SS_INT8, SS_UINT8 (8-bit signed/unsigned integers)
    SS_INT16, SS_UINT16 (16-bit signed/unsigned integers)
    SS_INT32, SS_UINT32 (32-bit signed/unsigned integers)
    SS_BOOLEAN 
    */
    
    // Set input ports
    if (!ssSetNumInputPorts(S, N_SIGNALS_SIM)) return;

    // Define direct feed through
    for (int i = 0; i < N_SIGNALS_SIM; i++) {
        ssSetInputPortDirectFeedThrough(S, i, 1); // Setting direct feedthrough to 0 for an input port is equivalent to saying that the corresponding input port signal is not used in mdlOutputs
    }

    // Define contiguous memory
    for (int i = 0; i < N_SIGNALS_SIM; i++) {
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    // Define widths and data types
    ssSetInputPortWidth(S, 0, MSG_N);   // msg_type
    ssSetInputPortDataType(S, 0, SS_UINT8);

    ssSetInputPortWidth(S, 1, ID_N);    // id
    ssSetInputPortDataType(S, 1, SS_UINT8);

    ssSetInputPortWidth(S, 2, 1);       // sim_status (int16)
    ssSetInputPortDataType(S, 2, SS_INT16);

    ssSetInputPortWidth(S, 3, 1);      // error_flag (int16)
    ssSetInputPortDataType(S, 3, SS_INT16);
    
    // Define one output port for serialized byte array
    if (!ssSetNumOutputPorts(S, 1)) return;

    ssSetOutputPortWidth(S, 0, SIM_MSG_SIZE);
    ssSetOutputPortDataType(S, 0, SS_UINT8);

    // Define sample times
    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    // **Check Input Port Connectivity**
    for (int i = 0; i < N_SIGNALS_SIM; i++) {
        if (!ssGetInputPortConnected(S, i)) {
            ssSetErrorStatus(S, "Missing input port connection. Serialization aborted.");
            return;
        }
    }

    for (int i = 0; i < N_SIGNALS_SIM; i++) {
        if (!ssGetInputPortRequiredContiguous(S, i)) {
            ssSetErrorStatus(S, "Input Port is required to be contiguous. Aborted.");
        }
    }

    // **Check Output Port Connectivity**
    if (!ssGetOutputPortConnected(S, 0)) {
        ssSetErrorStatus(S, "Output port is not connected. Data will not be transmitted.");
        return;
    }


    // Read each individual input from Simulink
    const uint8_T *msg_type = (const uint8_T *) ssGetInputPortSignal(S, 0);
    const uint8_T *id = (const uint8_T *) ssGetInputPortSignal(S, 1);
    const int16_T *sim_status = (const int16_T *) ssGetInputPortSignal(S, 2);
    const int16_T *error_flag = (const int16_T *) ssGetInputPortSignal(S, 3);

    // **Validation Checks**
    // if (isnan(*sim_status) || isnan(*error_flag)) {
    //     ssSetErrorStatus(S, "Invalid input: NaN detected in required field. Aborting serialization.");
    //     return;
    // }

    if (!(msg_type) || !(id)) {
        ssSetErrorStatus(S, "Invalid input: null detected in required field. Aborting serialization.");
        return;
    }

    // Populate SIM structure
    SIM sim = {0};

    strncpy(sim.msg_type, msg_type, MSG_N);
    strncpy(sim.id, id, ID_N);

    sim.sim_status = *sim_status;
    sim.error_flag = *error_flag;

    // Write to output
    uint8_T *serial = (uint8_T *) ssGetOutputPortSignal(S, 0);

    memcpy(serial, &sim, SIM_MSG_SIZE);
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif