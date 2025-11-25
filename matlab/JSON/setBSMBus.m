%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define Bus Elements
id_n = 16; % Hard-coded number of elements in BSM id string - Can be modified as needed
traj_n = 10; % Hard-coded number of elements in BSM trajectory vectors - Can be modified as needed

elements(1)  = Simulink.BusElement;
elements(1).Name = 'id';
elements(1).DataType = sprintf('uint8[%d]', id_n); % Simulink handles strings as uint8 arrays
elements(1).Dimensions = id_n;

elements(2)  = Simulink.BusElement;
elements(2).Name = 'current_utc_time';
elements(2).DataType = 'double';

elements(3)  = Simulink.BusElement;
elements(3).Name = 'latitude';
elements(3).DataType = 'double';

elements(4)  = Simulink.BusElement;
elements(4).Name = 'longitude';
elements(4).DataType = 'double';

elements(5)  = Simulink.BusElement;
elements(5).Name = 'rel_long_gap';
elements(5).DataType = 'double';

elements(6)  = Simulink.BusElement;
elements(6).Name = 'rel_lat_gap';
elements(6).DataType = 'double';

elements(7)  = Simulink.BusElement;
elements(7).Name = 'speed';
elements(7).DataType = 'double';

elements(8)  = Simulink.BusElement;
elements(8).Name = 'acceleration';
elements(8).DataType = 'double';

elements(9)  = Simulink.BusElement;
elements(9).Name = 'heading';
elements(9).DataType = 'double';

elements(10) = Simulink.BusElement;
elements(10).Name = 'heading_rate_change';
elements(10).DataType = 'double';

elements(11) = Simulink.BusElement;
elements(11).Name = 'error_flag';
elements(11).DataType = 'int16';

elements(12) = Simulink.BusElement;
elements(12).Name = 'elevation';
elements(12).DataType = 'double';

elements(13) = Simulink.BusElement;
elements(13).Name = 'position_accuracy';
elements(13).DataType = 'double';

elements(14) = Simulink.BusElement;
elements(14).Name = 'steering_angle';
elements(14).DataType = 'double';

elements(15) = Simulink.BusElement;
elements(15).Name = 'transmission_state';
elements(15).DataType = 'int16';

elements(16) = Simulink.BusElement;
elements(16).Name = 'braking_status';
elements(16).DataType = 'int16';

elements(17) = Simulink.BusElement;
elements(17).Name = 'turn_signal_status';
elements(17).DataType = 'int16';

elements(18) = Simulink.BusElement;
elements(18).Name = 'length';
elements(18).DataType = 'double';

elements(19) = Simulink.BusElement;
elements(19).Name = 'width';
elements(19).DataType = 'double';

% Path History
elements(20) = Simulink.BusElement;
elements(20).Name = 'path_history_n';
elements(20).DataType = 'uint16';

elements(21) = Simulink.BusElement;
elements(21).Name = 'path_history_time';
elements(21).DataType = sprintf('double[%d]', traj_n);
elements(21).Dimensions = traj_n;

elements(22) = Simulink.BusElement;
elements(22).Name = 'path_history_rel_long_pos';
elements(22).DataType = sprintf('double[%d]', traj_n);
elements(22).Dimensions = traj_n;

elements(23) = Simulink.BusElement;
elements(23).Name = 'path_history_rel_lat_pos';
elements(23).DataType = sprintf('double[%d]', traj_n);
elements(23).Dimensions = traj_n;

% Path Intentions
elements(24) = Simulink.BusElement;
elements(24).Name = 'path_intentions_n';
elements(24).DataType = 'uint16';

elements(25) = Simulink.BusElement;
elements(25).Name = 'path_intentions_time';
elements(25).DataType = sprintf('double[%d]', traj_n);
elements(25).Dimensions = traj_n;

elements(26) = Simulink.BusElement;
elements(26).Name = 'path_intentions_rel_long_pos';
elements(26).DataType = sprintf('double[%d]', traj_n);
elements(26).Dimensions = traj_n;

elements(27) = Simulink.BusElement;
elements(27).Name = 'path_intentions_rel_lat_pos';
elements(27).DataType = sprintf('double[%d]', traj_n);
elements(27).Dimensions = traj_n;

% Create Bus Object
busObj = Simulink.Bus;
busObj.Elements = elements;

% Assign Bus Object to Workspace
assignin('base', 'bsmBus', busObj);