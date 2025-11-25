%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Test Matlab script for Simulink model bsm_outputs
%%% Run after main.m
    
% Get bsm_outputs
if exist('bsm_out')
    disp('Starting BSM Serialization & Deserialization Tests...');

    current_utc_time = bsm_out.current_utc_time.Data;
    transmission_state = bsm_out.transmission_state.Data;
    heading = bsm_out.heading.Data;
    path_history_n = bsm_out.path_history_n.Data;
    path_history_time = bsm_out.path_history_time.Data;
    path_intentions_n = bsm_out.path_intentions_n.Data;
    path_intentions_time = bsm_out.path_intentions_time.Data;
    msg_type = bsm_out.msg_type.Data;
    id = bsm_out.id.Data;
    
    %%% Validate correctness
    assert(abs(current_utc_time - bsm.current_utc_time) < 1e-6, 'UTC Time mismatch!');
    assert(abs(transmission_state - bsm.transmission_state) < 1e-6, 'Transmission state mismatch!');
    assert(abs(heading - bsm.heading) < 1e-6, 'Heading mismatch!');
    
    assert(abs(path_history_n - bsm.path_history_n) < 1e-6, 'Path history n mismatch!');
    assert(all(isnan(path_history_time)), 'Path history mismatch!');
    assert(abs(path_intentions_n - bsm.path_intentions_n) < 1e-6, 'Path intentions n mismatch!');
    assert(all(path_intentions_time - bsm.path_intentions_time < 1e-4), 'Path intentions mismatch!');
    
    assert(strcmp(msg_type, bsm.msg_type), 'msg_type mismatch');
    assert(strcmp(id, bsm.id), 'msg_type mismatch');
    
    % Done
    disp('✅ BSM Serialization & Deserialization Tests Passed!');
end

if exist('sim_out')
    disp('Starting SIM Serialization & Deserialization Tests...');

    msg_type = sim_out.msg_type.Data;
    id = sim_out.id.Data;
    sim_status = sim_out.sim_status.Data;
    
    %%% Validate correctness
    assert(abs(sim_status - sim_msg.sim_status) < 1e-6, 'sim_status mismatch!');

    assert(strcmp(msg_type, sim_msg.msg_type), 'msg_type mismatch');
    assert(strcmp(id, sim_msg.id), 'msg_type mismatch');
    
    % Done
    disp('✅ SIM Serialization & Deserialization Tests Passed!');
end