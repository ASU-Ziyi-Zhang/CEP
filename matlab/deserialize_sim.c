///////////////////////////////////////
/// Tyler Ard                       ///
/// Vehicle Mobility Systems Group  ///
/// tard(at)anl(dot)gov             ///
///////////////////////////////////////

#define S_FUNCTION_NAME  deserialize_sim
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdint.h>
#include <string.h>

#include "serial_utils.h"

static void mdlInitializeSizes(SimStruct *S) {
    // Define function parameters
    ssSetNumSFcnParams(S, 0);

    // Define one input port for serialized byte array
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, SIM_MSG_SIZE);
    ssSetInputPortDataType(S, 0, SS_UINT8);

    // Define direct feed through
    for (int i = 0; i < 1; i++) {
        ssSetInputPortDirectFeedThrough(S, i, 1);  // Setting direct feedthrough to 0 for an input port is equivalent to saying that the corresponding input port signal is not used in mdlOutputs
    }
    
    // Define contiguous memory
    for (int i = 0; i < 1; i++) {
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    // Define separate output ports for each field in the BSM structure
    if (!ssSetNumOutputPorts(S, N_SIGNALS_SIM)) return;
    
    ssSetOutputPortWidth(S, 0, MSG_N);   // msg_type
    ssSetOutputPortDataType(S, 0, SS_UINT8);

    ssSetOutputPortWidth(S, 1, ID_N);    // id
    ssSetOutputPortDataType(S, 1, SS_UINT8);

    ssSetOutputPortWidth(S, 2, 1);       // sim_status (int16)
    ssSetOutputPortDataType(S, 2, SS_INT16);

    ssSetOutputPortWidth(S, 3, 1);      // error_flag (int16)
    ssSetOutputPortDataType(S, 3, SS_INT16);

    // Define sample times
    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    // **Check Input Port Connectivity**
    if (!ssGetInputPortConnected(S, 0)) {
        ssSetErrorStatus(S, "Input port is not connected. Data will not be transmitted.");
        return;
    }

    for (int i = 0; i < 1; i++) {
        if (!ssGetInputPortRequiredContiguous(S, i)) {
            ssSetErrorStatus(S, "Input Port is required to be contiguous. Deserialization aborted.");
        }
    }
    

    // **Check Output Port Connectivity**
    for (int i = 0; i < N_SIGNALS_SIM; i++) {
        if (!ssGetOutputPortConnected(S, i)) {
            ssSetErrorStatus(S, "Missing output port connection. Deserialization aborted.");
            return;
        }
    }

    // Read serialized input from Simulink
    const uint8_T *serial = (const uint8_T *) ssGetInputPortSignalPtrs(S, 0);

    // Deserialize data into the SIM struct
    SIM sim;
    memcpy(&sim, serial, sizeof(SIM));

    // Assign outputs
    uint8_T *msg_type = (uint8_T *) ssGetOutputPortSignal(S, 0);
    uint8_T *id = (uint8_T *) ssGetOutputPortSignal(S, 1);
    int16_T *sim_status = (int16_T *) ssGetOutputPortSignal(S, 2);
    int16_T *error_flag = (int16_T *) ssGetOutputPortSignal(S, 3);
    
    // Assign struct values to outputs
    strncpy(msg_type, sim.msg_type, MSG_N);
    strncpy(id, sim.id, ID_N);

    *sim_status = sim.sim_status;
    *error_flag = sim.error_flag;
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif