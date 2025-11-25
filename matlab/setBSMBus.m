%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Define size for path history and path intentions vectors
VECTOR_SIZE = 10; % Can be modified as needed

% Define Bus Elements for BSM Message
elems(1) = Simulink.BusElement;
elems(1).Name = 'current_utc_time';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'latitude';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'longitude';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'speed';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'acceleration';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'heading';
elems(6).DataType = 'double';

elems(7) = Simulink.BusElement;
elems(7).Name = 'heading_rate_change';
elems(7).DataType = 'double';

elems(8) = Simulink.BusElement;
elems(8).Name = 'error_flag';
elems(8).DataType = 'uint8';

% Define Path History struct
historyElements(1) = Simulink.BusElement;
historyElements(1).Name = 'rel_time';
historyElements(1).DataType = 'double';
historyElements(1).Dimensions = [VECTOR_SIZE,1];

historyElements(2) = Simulink.BusElement;
historyElements(2).Name = 'lateral_deviation';
historyElements(2).DataType = 'double';
historyElements(2).Dimensions = [VECTOR_SIZE,1];

historyElements(3) = Simulink.BusElement;
historyElements(3).Name = 'longitudinal_deviation';
historyElements(3).DataType = 'double';
historyElements(3).Dimensions = [VECTOR_SIZE,1];

pathHistoryBus = Simulink.Bus;
pathHistoryBus.Elements = historyElements;

% Define Path Intentions struct
intentionsElements(1) = Simulink.BusElement;
intentionsElements(1).Name = 'rel_time';
intentionsElements(1).DataType = 'double';
intentionsElements(1).Dimensions = [VECTOR_SIZE,1];

intentionsElements(2) = Simulink.BusElement;
intentionsElements(2).Name = 'lateral_deviation';
intentionsElements(2).DataType = 'double';
intentionsElements(2).Dimensions = [VECTOR_SIZE,1];

intentionsElements(3) = Simulink.BusElement;
intentionsElements(3).Name = 'longitudinal_deviation';
intentionsElements(3).DataType = 'double';
intentionsElements(3).Dimensions = [VECTOR_SIZE,1];

pathIntentionsBus = Simulink.Bus;
pathIntentionsBus.Elements = intentionsElements;

% Add Path History and Path Intentions to Main BSM Bus
elems(9) = Simulink.BusElement;
elems(9).Name = 'path_history';
elems(9).DataType = 'Bus';
elems(9).BusType = 'pathHistoryBus';

elems(10) = Simulink.BusElement;
elems(10).Name = 'path_intentions';
elems(10).DataType = 'Bus';
elems(10).BusType = 'pathIntentionsBus';

% Create BSM Bus Object
BSM_Bus = Simulink.Bus;
BSM_Bus.Elements = elems;

% Assign to Base Workspace
assignin('base', 'VECTOR_SIZE', VECTOR_SIZE);
assignin('base', 'BSM_Bus', BSM_Bus);
assignin('base', 'pathHistoryBus', pathHistoryBus);
assignin('base', 'pathIntentionsBus', pathIntentionsBus);