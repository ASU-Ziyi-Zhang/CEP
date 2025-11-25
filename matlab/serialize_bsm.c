///////////////////////////////////////
/// Tyler Ard                       ///
/// Vehicle Mobility Systems Group  ///
/// tard(at)anl(dot)gov             ///
///////////////////////////////////////

#define S_FUNCTION_NAME  serialize_bsm
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
    if (!ssSetNumInputPorts(S, N_SIGNALS_BSM)) return;

    // Define direct feed through
    for (int i = 0; i < N_SIGNALS_BSM; i++) {
        const int ft = 1; // Setting direct feedthrough to 0 for an input port is equivalent to saying that the corresponding input port signal is not used in mdlOutputs
        ssSetInputPortDirectFeedThrough(S, i, ft);
    }

    // Define contiguous memory
    for (int i = 0; i < N_SIGNALS_BSM; i++) {
        const int co = 1;
        ssSetInputPortRequiredContiguous(S, i, co);
    }

    // Define widths and data types
    ssSetInputPortWidth(S, 0, MSG_N);   // msg_type
    ssSetInputPortWidth(S, 1, ID_N);    // id
    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortDataType(S, 1, SS_UINT8);

    ssSetInputPortWidth(S, 2, 1);       // current_utc_time
    ssSetInputPortWidth(S, 3, 1);       // latitude
    ssSetInputPortWidth(S, 4, 1);       // longitude
    ssSetInputPortWidth(S, 5, 1);       // rel_long_gap
    ssSetInputPortWidth(S, 6, 1);       // rel_lat_gap
    ssSetInputPortWidth(S, 7, 1);       // speed
    ssSetInputPortWidth(S, 8, 1);       // acceleration
    ssSetInputPortWidth(S, 9, 1);       // heading
    ssSetInputPortWidth(S, 10, 1);      // heading_rate_change

    ssSetInputPortWidth(S, 11, 1);      // error_flag (int16)
    ssSetInputPortDataType(S, 11, SS_INT16);

    ssSetInputPortWidth(S, 12, 1);      // elevation
    ssSetInputPortWidth(S, 13, 1);      // position_accuracy
    ssSetInputPortWidth(S, 14, 1);      // steering_angle

    ssSetInputPortWidth(S, 15, 1);      // transmission_state (int16)
    ssSetInputPortWidth(S, 16, 1);      // braking_status (int16)
    ssSetInputPortWidth(S, 17, 1);      // turn_signal_status (int16)
    ssSetInputPortDataType(S, 15, SS_INT16);
    ssSetInputPortDataType(S, 16, SS_INT16);
    ssSetInputPortDataType(S, 17, SS_INT16);

    ssSetInputPortWidth(S, 18, 1);      // length
    ssSetInputPortWidth(S, 19, 1);      // width

    ssSetInputPortWidth(S, 20, 1);      // path_history_n (uint16)
    ssSetInputPortDataType(S, 20, SS_UINT16);

    ssSetInputPortWidth(S, 21, TRAJ_N); // path_history_time
    ssSetInputPortWidth(S, 22, TRAJ_N); // path_history_rel_long_pos
    ssSetInputPortWidth(S, 23, TRAJ_N); // path_history_rel_lat_pos
    
    ssSetInputPortWidth(S, 24, 1);      // path_intentions_n (uint16)
    ssSetInputPortDataType(S, 24, SS_UINT16);

    ssSetInputPortWidth(S, 25, TRAJ_N); // path_intentions_time
    ssSetInputPortWidth(S, 26, TRAJ_N); // path_intentions_rel_long_pos
    ssSetInputPortWidth(S, 27, TRAJ_N); // path_intentions_rel_lat_pos

    // Define one output port for serialized byte array
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, BSM_MSG_SIZE);
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
    for (int i = 0; i < N_SIGNALS_BSM; i++) {
        if (!ssGetInputPortConnected(S, i)) {
            ssSetErrorStatus(S, "Missing input port connection. Serialization aborted.");
            return;
        }
    }

    for (int i = 0; i < N_SIGNALS_BSM; i++) {
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
    const real_T *current_utc_time = (const real_T *) ssGetInputPortSignal(S, 2);
    const real_T *latitude = (const real_T *) ssGetInputPortSignal(S, 3);
    const real_T *longitude = (const real_T *) ssGetInputPortSignal(S, 4);
    const real_T *rel_long_gap = (const real_T *) ssGetInputPortSignal(S, 5);
    const real_T *rel_lat_gap = (const real_T *) ssGetInputPortSignal(S, 6);
    const real_T *speed = (const real_T *) ssGetInputPortSignal(S, 7);
    const real_T *acceleration = (const real_T *) ssGetInputPortSignal(S, 8);
    const real_T *heading = (const real_T *) ssGetInputPortSignal(S, 9);
    const real_T *heading_rate_change = (const real_T *) ssGetInputPortSignal(S, 10);
    const int16_T *error_flag = (const int16_T *) ssGetInputPortSignal(S, 11);
    const real_T *elevation = (const real_T *) ssGetInputPortSignal(S, 12);
    const real_T *position_accuracy = (const real_T *) ssGetInputPortSignal(S, 13);
    const real_T *steering_angle = (const real_T *) ssGetInputPortSignal(S, 14);
    const int16_T *transmission_state = (const int16_T *) ssGetInputPortSignal(S, 15);
    const int16_T *braking_status = (const int16_T *) ssGetInputPortSignal(S, 16);
    const int16_T *turn_signal_status = (const int16_T *) ssGetInputPortSignal(S, 17);
    const real_T *length = (const real_T *) ssGetInputPortSignal(S, 18);
    const real_T *width = (const real_T *) ssGetInputPortSignal(S, 19);
    const uint16_T *path_history_n = (const uint16_T *) ssGetInputPortSignal(S, 20);
    const real_T *path_history_time = (const real_T *) ssGetInputPortSignal(S, 21);
    const real_T *path_history_rel_long_pos = (const real_T *) ssGetInputPortSignal(S, 22);
    const real_T *path_history_rel_lat_pos = (const real_T *) ssGetInputPortSignal(S, 23);
    const uint16_T *path_intentions_n = (const uint16_T *) ssGetInputPortSignal(S, 24);
    const real_T *path_intentions_time = (const real_T *) ssGetInputPortSignal(S, 25);
    const real_T *path_intentions_rel_long_pos = (const real_T *) ssGetInputPortSignal(S, 26);
    const real_T *path_intentions_rel_lat_pos = (const real_T *) ssGetInputPortSignal(S, 27);


    // **Validation Checks**
    // if (isnan(*current_utc_time) || isnan(*latitude) || isnan(*longitude) || 
    //     isnan(*speed) || isnan(*acceleration) || isnan(*heading)) {
    //     ssSetErrorStatus(S, "Invalid input: NaN detected in required field. Aborting serialization.");
    //     return;
    // }

    if (!(current_utc_time) || !(latitude) || !(longitude) || 
        !(speed) || !(acceleration) || !(heading)) {
        ssSetErrorStatus(S, "Invalid input: null detected in required field. Aborting serialization.");
        return;
    }

    if (BSM_MSG_SIZE != sizeof(BSM)) {
        ssSetErrorStatus(S, "Invalid output: size mismatch between BSM struct size and BSM_MSG_SIZE definition. Aborting serialization.");
        return;
    }

    // Populate BSM structure
    BSM bsm = {0};

    strncpy(bsm.msg_type, msg_type, MSG_N);
    strncpy(bsm.id, id, ID_N);

    bsm.current_utc_time = *current_utc_time;
    bsm.latitude = *latitude;
    bsm.longitude = *longitude;
    bsm.rel_long_gap = *rel_long_gap;
    bsm.rel_lat_gap = *rel_lat_gap;
    bsm.speed = *speed;
    bsm.acceleration = *acceleration;
    bsm.heading = *heading;
    bsm.heading_rate_change = *heading_rate_change;

    bsm.error_flag = *error_flag;
    bsm.elevation = *elevation;
    bsm.position_accuracy = *position_accuracy;
    bsm.steering_angle = *steering_angle;
    bsm.transmission_state = *transmission_state;
    bsm.braking_status = *braking_status;
    bsm.turn_signal_status = *turn_signal_status;
    bsm.length = *length;
    bsm.width = *width;

    bsm.path_history_n = *path_history_n;
    bsm.path_intentions_n = *path_intentions_n;

    // Copy array values within bounds
    for (size_t i = 0; i < TRAJ_N; i++) {
        bsm.path_history_time[i] = path_history_time[i];
        bsm.path_history_rel_long_pos[i] = path_history_rel_long_pos[i];
        bsm.path_history_rel_lat_pos[i] = path_history_rel_lat_pos[i];

        bsm.path_intentions_time[i] = path_intentions_time[i];
        bsm.path_intentions_rel_long_pos[i] = path_intentions_rel_long_pos[i];
        bsm.path_intentions_rel_lat_pos[i] = path_intentions_rel_lat_pos[i];
    }

    // Write to output
    uint8_T *serial = (uint8_T *) ssGetOutputPortSignal(S, 0);

    memcpy(serial, &bsm, BSM_MSG_SIZE);
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif