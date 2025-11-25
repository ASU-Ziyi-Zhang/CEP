///////////////////////////////////////
/// Tyler Ard                       ///
/// Vehicle Mobility Systems Group  ///
/// tard(at)anl(dot)gov             ///
///////////////////////////////////////

#define N_SIGNALS_BSM 28 // Number of total BMS signals
#define N_SIGNALS_SIM 4 // Number of total SIM signals

#define MSG_N 31 // Number of msg type chars
#define ID_N 31  // Number of ID chars

#define TRAJ_N 10  // Number of trajectory points in BSM

typedef struct {
    uint8_T msg_type[MSG_N];
    uint8_T id[ID_N];
    real_T current_utc_time;
    real_T latitude, longitude, rel_long_gap, rel_lat_gap;
    real_T speed, acceleration, heading, heading_rate_change;
    int16_T error_flag;
    real_T elevation, position_accuracy, steering_angle;
    int16_T transmission_state, braking_status, turn_signal_status;
    real_T length, width;
    uint16_T path_history_n;
    real_T path_history_time[TRAJ_N], path_history_rel_long_pos[TRAJ_N], path_history_rel_lat_pos[TRAJ_N];
    uint16_T path_intentions_n;
    real_T path_intentions_time[TRAJ_N], path_intentions_rel_long_pos[TRAJ_N], path_intentions_rel_lat_pos[TRAJ_N];
} BSM; // Should have N_SIGNALS_BSM number of fields

#define BSM_MSG_SIZE sizeof(BSM)

typedef struct {
    uint8_T msg_type[MSG_N];
    uint8_T id[ID_N];
    int16_T sim_status;
    int16_T error_flag;
} SIM;

#define SIM_MSG_SIZE sizeof(SIM)

void forceCharArrayN(const char *input, char *output, size_t n);






/*
#define BSM_MSG_SIZE (              \
    MSG_N +                         \
    ID_N +                          \
    (1+4+4+3+2) * sizeof(real_T) +  \
    4 * sizeof(int16_T) +           \
    2 * sizeof(uint16_T) +          \
    2*3 * TRAJ_N * sizeof(real_T)   \
)
*/