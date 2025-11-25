///////////////////////////////////////
/// Tyler Ard                       ///
/// Vehicle Mobility Systems Group  ///
/// tard(at)anl(dot)gov             ///
///////////////////////////////////////

#define S_FUNCTION_NAME  deserialize_bsm
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
    ssSetInputPortWidth(S, 0, BSM_MSG_SIZE);
    ssSetInputPortDataType(S, 0, SS_UINT8);

    // Define direct feed through
    for (int i = 0; i < 1; i++) {
        const int FEEDTHROUGH = 1; // Setting direct feedthrough to 0 for an input port is equivalent to saying that the corresponding input port signal is not used in mdlOutputs
        ssSetInputPortDirectFeedThrough(S, i, FEEDTHROUGH);
    }
    
    // Define contiguous memory
    for (int i = 0; i < 1; i++) {
        const int co = 1;
        ssSetInputPortRequiredContiguous(S, i, co);
    }

    // Define separate output ports for each field in the BSM structure
    if (!ssSetNumOutputPorts(S, N_SIGNALS_BSM)) return;
    
    ssSetOutputPortWidth(S, 0, MSG_N);   // msg_type
    ssSetOutputPortWidth(S, 1, ID_N);    // id
    ssSetOutputPortDataType(S, 0, SS_UINT8);
    ssSetOutputPortDataType(S, 1, SS_UINT8);

    ssSetOutputPortWidth(S, 2, 1);       // current_utc_time
    ssSetOutputPortWidth(S, 3, 1);       // latitude
    ssSetOutputPortWidth(S, 4, 1);       // longitude
    ssSetOutputPortWidth(S, 5, 1);       // rel_long_gap
    ssSetOutputPortWidth(S, 6, 1);       // rel_lat_gap
    ssSetOutputPortWidth(S, 7, 1);       // speed
    ssSetOutputPortWidth(S, 8, 1);       // acceleration
    ssSetOutputPortWidth(S, 9, 1);       // heading
    ssSetOutputPortWidth(S, 10, 1);      // heading_rate_change
    
    ssSetOutputPortWidth(S, 11, 1);      // error_flag (int16)
    ssSetOutputPortDataType(S, 11, SS_INT16);

    ssSetOutputPortWidth(S, 12, 1);      // elevation
    ssSetOutputPortWidth(S, 13, 1);      // position_accuracy
    ssSetOutputPortWidth(S, 14, 1);      // steering_angle
    
    ssSetOutputPortWidth(S, 15, 1);      // transmission_state (int16)
    ssSetOutputPortWidth(S, 16, 1);      // braking_status (int16)
    ssSetOutputPortWidth(S, 17, 1);      // turn_signal_status (int16)
    ssSetOutputPortDataType(S, 15, SS_INT16);
    ssSetOutputPortDataType(S, 16, SS_INT16);
    ssSetOutputPortDataType(S, 17, SS_INT16);

    ssSetOutputPortWidth(S, 18, 1);      // length
    ssSetOutputPortWidth(S, 19, 1);      // width
    
    ssSetOutputPortWidth(S, 20, 1);      // path_history_n (uint16)
    ssSetOutputPortDataType(S, 20, SS_UINT16);
    
    ssSetOutputPortWidth(S, 21, TRAJ_N); // path_history_time
    ssSetOutputPortWidth(S, 22, TRAJ_N); // path_history_rel_long_pos
    ssSetOutputPortWidth(S, 23, TRAJ_N); // path_history_rel_lat_pos
    
    ssSetOutputPortWidth(S, 24, 1);      // path_intentions_n (uint16)
    ssSetOutputPortDataType(S, 24, SS_UINT16);
    
    ssSetOutputPortWidth(S, 25, TRAJ_N); // path_intentions_time
    ssSetOutputPortWidth(S, 26, TRAJ_N); // path_intentions_rel_long_pos
    ssSetOutputPortWidth(S, 27, TRAJ_N); // path_intentions_rel_lat_pos

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
    for (int i = 0; i < N_SIGNALS_BSM; i++) {
        if (!ssGetOutputPortConnected(S, i)) {
            ssSetErrorStatus(S, "Missing output port connection. Deserialization aborted.");
            return;
        }
    }

    // Read serialized input from Simulink
    const uint8_T *serial = (const uint8_T *) ssGetInputPortSignalPtrs(S, 0);

    // Deserialize data into the BSM struct
    BSM bsm;
    memcpy(&bsm, serial, sizeof(BSM));

    // Assign outputs
    uint8_T *msg_type = (uint8_T *) ssGetOutputPortSignal(S, 0);
    uint8_T *id = (uint8_T *) ssGetOutputPortSignal(S, 1);
    real_T *current_utc_time = ssGetOutputPortRealSignal(S, 2);
    real_T *latitude = ssGetOutputPortRealSignal(S, 3);
    real_T *longitude = ssGetOutputPortRealSignal(S, 4);
    real_T *rel_long_gap = ssGetOutputPortRealSignal(S, 5);
    real_T *rel_lat_gap = ssGetOutputPortRealSignal(S, 6);
    real_T *speed = ssGetOutputPortRealSignal(S, 7);
    real_T *acceleration = ssGetOutputPortRealSignal(S, 8);
    real_T *heading = ssGetOutputPortRealSignal(S, 9);
    real_T *heading_rate_change = ssGetOutputPortRealSignal(S, 10);
    int16_T *error_flag = (int16_T *) ssGetOutputPortSignal(S, 11);
    real_T *elevation = ssGetOutputPortRealSignal(S, 12);
    real_T *position_accuracy = ssGetOutputPortRealSignal(S, 13);
    real_T *steering_angle = ssGetOutputPortRealSignal(S, 14);
    int16_T *transmission_state = (int16_T *) ssGetOutputPortSignal(S, 15);
    int16_T *braking_status = (int16_T *) ssGetOutputPortSignal(S, 16);
    int16_T *turn_signal_status = (int16_T *) ssGetOutputPortSignal(S, 17);
    real_T *length = ssGetOutputPortRealSignal(S, 18);
    real_T *width = ssGetOutputPortRealSignal(S, 19);
    uint16_T *path_history_n = (uint16_T *) ssGetOutputPortSignal(S, 20);
    real_T *path_history_time = ssGetOutputPortRealSignal(S, 21);
    real_T *path_history_rel_long_pos = ssGetOutputPortRealSignal(S, 22);
    real_T *path_history_rel_lat_pos = ssGetOutputPortRealSignal(S, 23);
    uint16_T *path_intentions_n = (uint16_T *) ssGetOutputPortSignal(S, 24);
    real_T *path_intentions_time = ssGetOutputPortRealSignal(S, 25);
    real_T *path_intentions_rel_long_pos = ssGetOutputPortRealSignal(S, 26);
    real_T *path_intentions_rel_lat_pos = ssGetOutputPortRealSignal(S, 27);

    // Assign struct values to outputs
    strncpy(msg_type, bsm.msg_type, MSG_N);
    strncpy(id, bsm.id, ID_N);

    *current_utc_time = bsm.current_utc_time;
    *latitude = bsm.latitude;
    *longitude = bsm.longitude;
    *rel_long_gap = bsm.rel_long_gap;
    *rel_lat_gap = bsm.rel_lat_gap;
    *speed = bsm.speed;
    *acceleration = bsm.acceleration;
    *heading = bsm.heading;
    *heading_rate_change = bsm.heading_rate_change;
    *error_flag = bsm.error_flag;
    *elevation = bsm.elevation;
    *position_accuracy = bsm.position_accuracy;
    *steering_angle = bsm.steering_angle;
    *transmission_state = bsm.transmission_state;
    *braking_status = bsm.braking_status;
    *turn_signal_status = bsm.turn_signal_status;
    *length = bsm.length;
    *width = bsm.width;

    *path_history_n = bsm.path_history_n;
    memcpy(path_history_time, bsm.path_history_time, TRAJ_N * sizeof(real_T));
    memcpy(path_history_rel_long_pos, bsm.path_history_rel_long_pos, TRAJ_N * sizeof(real_T));
    memcpy(path_history_rel_lat_pos, bsm.path_history_rel_lat_pos, TRAJ_N * sizeof(real_T));
    
    *path_intentions_n = bsm.path_intentions_n;
    memcpy(path_intentions_time, bsm.path_intentions_time, TRAJ_N * sizeof(real_T));
    memcpy(path_intentions_rel_long_pos, bsm.path_intentions_rel_long_pos, TRAJ_N * sizeof(real_T));
    memcpy(path_intentions_rel_lat_pos, bsm.path_intentions_rel_lat_pos, TRAJ_N * sizeof(real_T));
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif