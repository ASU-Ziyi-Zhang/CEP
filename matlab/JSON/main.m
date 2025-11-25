%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Main Matlab script implementation of a basic UDP client sending BSM to
%%% a remote server
%%%
%%% Creates a BSM object, then serializes using JSON, and then sends via UDP

clearvars; clc

setBSMBus

%%% Create UDP socket
IP_addr = "127.0.0.1";
port = 12345;
% socket = udpport("IPv4", "LocalPort", port); % Specify local listening port - Works if Toolbox installed :)

%%% Set client behavior parameters
timeout = 0.1; % Listening duration in seconds
sendInterval = 2; % Send a message every 2 seconds

%%% Make BSM template
msg = BSM();

% Start a listening and sending loop for n seconds
startTime = tic;
while toc(startTime) < timeout
    %%% Make the BSM - use some default 'sensor' values of vehicle INSS
    latitude = -4.8;
    longitude = 0.0;
    speed = 0.0;
    acceleration = 0.0;
    heading = pi/2;
    headingRateChange = 0.0;

    % msg = msg(latitude, longitude, speed, acceleration, heading, headingRateChange);

    path_intentions_times = linspace(0.0, 10.0, 10); % VECTOR_SIZE set in the setBSMBus.m filematlab
    path_intentions_pos_deviations = linspace(20.0, 40.0, 10); % VECTOR_SIZE set in the setBSMBus.m filematlab
    
    msg = msg(latitude, longitude, speed, acceleration, heading, headingRateChange, path_intentions_times, path_intentions_pos_deviations);
    
    %%% Convert to a json string then serialize
    json_str = setJSONfromBSM(msg);
    
    % Send JSON message to Server
    % write(socket, uint8(json_str), IP_addr, port); % Replace IP and port as needed
    % 
    % fprintf("Sent message: %s\n", jsonMessage);

    %%% Check for reply from Server
    pause_time = 0.5;
    pause(pause_time); % Short pause before checking buffer -- In real implementation don't do this 

    % if socket.NumBytesAvailable > 0
    %     server_reply = read(socket, socket.NumBytesAvailable, "uint8");
    %     json_str = char(server_reply'); % Convert bytes to string
    % 
    %     % Decode server reply
    %     pv_bsm = getJSONsimple(json_str);
    % 
    %     fprintf("Received reply: %s\n", json_str);
    % end

    pv_bsm = getBSMfromJSON(json_str);

    %%% Wait before repeating
    pause(sendInterval - pause_time); % Adjust for earlier pause
end

% Close UDP connection
% clear socket;

% Done
disp(pv_bsm)
disp(pv_bsm.path_intentions)
disp(pv_bsm.path_intentions.rel_time)
disp(pv_bsm.path_intentions.longitudinal_deviation)

fprintf('Done.\n')
