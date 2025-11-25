%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function jsonData = setJSONfromBSM(bsmMessage)
    %#codegen
    % Converts a BSM message struct to JSON format for Simulink & C Code Generation.
    %
    % Inputs:
    %   bsmMessage - Struct containing the BSM data with fixed fields.
    %
    % Outputs:
    %   jsonData - JSON formatted string.

    % Validate input type
    if ~isstruct(bsmMessage)
        error('Input must be a struct containing BSM data.');
    end
    
    % Ensure path history and path intentions are correctly formatted
    if ~isfield(bsmMessage, 'path_history')
        bsmMessage.path_history = struct('rel_time', [], 'lateral_deviation', [], 'longitudinal_deviation', []);
    end
    if ~isfield(bsmMessage, 'path_intentions')
        bsmMessage.path_intentions = struct('rel_time', [], 'lateral_deviation', [], 'longitudinal_deviation', []);
    end
    
    % % Convert path history and path intentions to JSON-compatible format
    % pathHistoryJSON = sprintf('{\"rel_time\":[%s],\"lateral_deviation\":[%s],\"longitudinal_deviation\":[%s]}', ...
    %     num2str(bsmMessage.path_history.rel_time, '%.6f,'), ...
    %     num2str(bsmMessage.path_history.lateral_deviation, '%.6f,'), ...
    %     num2str(bsmMessage.path_history.longitudinal_deviation, '%.6f,'));
    % 
    % pathIntentionsJSON = sprintf('{\"rel_time\":[%s],\"lateral_deviation\":[%s],\"longitudinal_deviation\":[%s]}', ...
    %     num2str(bsmMessage.path_intentions.rel_time, '%.6f,'), ...
    %     num2str(bsmMessage.path_intentions.lateral_deviation, '%.6f,'), ...
    %     num2str(bsmMessage.path_intentions.longitudinal_deviation, '%.6f,'));
    % 
    % % Manually construct the full JSON string
    % jsonData = sprintf('{\"current_utc_time\":%.6f, \"latitude\":%.6f, \"longitude\":%.6f, \"speed\":%.2f, \"acceleration\":%.2f, \"heading\":%.6f, \"heading_rate_change\":%.6f, \"error_flag\":\"%s\", \"path_history\":%s, \"path_intentions\":%s}', ...
    %     bsmMessage.current_utc_time, bsmMessage.latitude, bsmMessage.longitude, ...
    %     bsmMessage.speed, bsmMessage.acceleration, bsmMessage.heading, ...
    %     bsmMessage.heading_rate_change, bsmMessage.error_flag, ...
    %     pathHistoryJSON, pathIntentionsJSON);

    % Convert path history and path intentions to JSON-compatible format
    pathHistoryJSON = sprintf('{\"rel_time\":[%s],\"lateral_deviation\":[%s],\"longitudinal_deviation\":[%s]}', ...
    formatArrayForJSON(bsmMessage.path_history.rel_time, '%.6f'), ...
    formatArrayForJSON(bsmMessage.path_history.lateral_deviation, '%.6f'), ...
    formatArrayForJSON(bsmMessage.path_history.longitudinal_deviation, '%.6f'));

    pathIntentionsJSON = sprintf('{\"rel_time\":[%s],\"lateral_deviation\":[%s],\"longitudinal_deviation\":[%s]}', ...
    formatArrayForJSON(bsmMessage.path_intentions.rel_time, '%.6f'), ...
    formatArrayForJSON(bsmMessage.path_intentions.lateral_deviation, '%.6f'), ...
    formatArrayForJSON(bsmMessage.path_intentions.longitudinal_deviation, '%.6f'));

    % Manually construct the full JSON string
    jsonData = sprintf('{\"current_utc_time\":%.6f, \"latitude\":%.6f, \"longitude\":%.6f, \"speed\":%.2f, \"acceleration\":%.2f, \"heading\":%.6f, \"heading_rate_change\":%.6f, \"error_flag\":\"%s\", \"path_history\":%s, \"path_intentions\":%s}', ...
    bsmMessage.current_utc_time, bsmMessage.latitude, bsmMessage.longitude, ...
    bsmMessage.speed, bsmMessage.acceleration, bsmMessage.heading, ...
    bsmMessage.heading_rate_change, bsmMessage.error_flag, ...
    pathHistoryJSON, pathIntentionsJSON);
end

% Helper function for formatting arrays without num2str
function formattedStr = formatArrayForJSON(array, formatSpec)
    %#codegen
    % Converts a numeric array into a JSON-compatible formatted string.
    
    formattedStr = '';
    for i = 1:length(array)
        formattedStr = [formattedStr, sprintf(formatSpec, array(i))];
        if i < length(array)
            formattedStr = [formattedStr, ',']; % Add comma separator
        end
    end
end
