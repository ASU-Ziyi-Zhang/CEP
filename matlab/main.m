%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Main Matlab script implementation of a basic UDP client sending BSM-likes to
%%% a remote server
%%%
%%% Creates a BSM object, then serializes into a byte array
%%% Creates a SIM object, then serializes into a byte array

clearvars; clc

is_simming = true;

bsm_sys_name = 'main_bsm_sl';
sim_sys_name = 'main_sim_sl';

%%% Compile the serialization and deserialization S-functions
% cc = mex.getCompilerConfigurations('C', 'Selected');
% if contains(cc.Name, 'MinGW')
%     error('The selected MinGW compiler is not supported for the MEX functions. Try the MSVC C compiler.')
% end

% Pass -g command to build with debugging info
disp('Compiling MEX functions...')

mex -g serialize_bsm.c
mex -g deserialize_bsm.c

mex -g serialize_sim.c
mex -g deserialize_sim.c

%%% Define sample structured test data - MATCH these with the serial_utils.h #define
msg_n = 31;
id_n = 31;
traj_n = 10;

% Required
bsm.msg_type = "bsm"; % String
bsm.id = "Ego_1"; % String

bsm.current_utc_time = 3612635768669.0;
bsm.latitude = -4.5;
bsm.longitude = 0.0;

bsm.speed = 0.0;
bsm.acceleration = 0.0;
bsm.heading = 90.0;
bsm.heading_rate_change = 0.0;
bsm.error_flag = int16(0); % (0) OK , (1) Error

% From sim describing surrounding vehicles
bsm.rel_long_gap = nan;
bsm.rel_lat_gap = nan;

% Optionals
bsm.elevation = 0.0;
bsm.position_accuracy = 1.5;
bsm.steering_angle = 0.0;
bsm.transmission_state = int16(0);
bsm.braking_status = int16(0);
bsm.turn_signal_status = int16(0);
bsm.length = 4.2;
bsm.width = 1.9;

% Optionals CDA-B
bsm.path_history_time = linspace(nan, nan, traj_n); % For testing make this nan
bsm.path_history_rel_long_pos = linspace(nan, nan, traj_n);
bsm.path_history_rel_lat_pos = linspace(nan, nan, traj_n);
bsm.path_history_n = uint16( nnz(bsm.path_history_time) );

bsm.path_intentions_time = linspace(0, 12, traj_n); % For testing make this finite
bsm.path_intentions_rel_long_pos = linspace(50, 65, traj_n);
bsm.path_intentions_rel_lat_pos = linspace(1, 1, traj_n);
bsm.path_intentions_n = uint16( nnz(bsm.path_intentions_time) );

%%% 
% SIM_STATUS:
% OFFLINE = -2 # Shutdown or will shutdown and will cease communication
% NOT_READY = -1 # Communicating and preparing but not ready yet
% WAITING = 0 # Communicating and ready but waiting for confirmation to start
% RUNNING = 1 # Communicating and ready, has received confirmation to start
%
% Server will attempt to start when all clients it is expecting to connect
% publish at least the WAITING flag
% Server will stop when OFFLINE flag is received from a client
% Server will send an OFFLINE flag when it ends, due to a detected client disconnect, error in simulation, natural close of scenario, etc

sim_msg.msg_type = "sim"; % String
sim_msg.id = bsm.id; % String, same ego vehicle so same id
sim_msg.sim_status = int16(0); % Update this during experiment process
sim_msg.error_flag = int16(0);

%%% Run serialization S-Function
%%% Run deserialization S-Function
if is_simming
    %%% Sim
    % BSM sim
    open_system(bsm_sys_name);
    bsm_out = sim(bsm_sys_name);

    % SIM sim
    open_system(sim_sys_name);
    sim_out = sim(sim_sys_name);

    %%% Test
    test

end