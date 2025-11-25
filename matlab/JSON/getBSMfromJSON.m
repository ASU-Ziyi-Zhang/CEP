%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function structData = getBSMfromJSON(jsonData)
    %#codegen
    % Parses JSON string manually into a structured format for Simulink & MATLAB CodeGen.

    % Ensure input is a string
    assert(ischar(jsonData) || isstring(jsonData), 'Input must be a valid JSON string.');

    % Initialize default struct with expected fields
    max_vector_len = 100;

    structData = struct( ...
        'current_utc_time', 0.0, ...
        'latitude', 0.0, ...
        'longitude', 0.0, ...
        'speed', 0.0, ...
        'acceleration', 0.0, ...
        'heading', 0.0, ...
        'heading_rate_change', 0.0, ...
        'error_flag', "OK", ...
        'path_history', struct('rel_time', nan(max_vector_len,1), 'lateral_deviation', nan(max_vector_len,1), 'longitudinal_deviation', nan(max_vector_len,1)), ...
        'path_intentions', struct('rel_time', nan(max_vector_len,1), 'lateral_deviation', nan(max_vector_len,1), 'longitudinal_deviation', nan(max_vector_len,1)) ...
    );

    % Extract numeric values using manual parsing (C/C++ compatible)
    structData.current_utc_time = extractNumeric(jsonData, '"current_utc_time":');
    structData.latitude = extractNumeric(jsonData, '"latitude":');
    structData.longitude = extractNumeric(jsonData, '"longitude":');
    structData.speed = extractNumeric(jsonData, '"speed":');
    structData.acceleration = extractNumeric(jsonData, '"acceleration":');
    structData.heading = extractNumeric(jsonData, '"heading":');
    structData.heading_rate_change = extractNumeric(jsonData, '"heading_rate_change":');

    % Extract error flag as a character string
    structData.error_flag = extractString(jsonData, '"error_flag":"');

    % Extract path history arrays
    path_history = extractNestedJson(jsonData, '"path_history":');
    
    structData.path_history.rel_time = extractArray(path_history, '"rel_time":[', max_vector_len);
    structData.path_history.lateral_deviation = extractArray(path_history, '"lateral_deviation":[', max_vector_len);
    structData.path_history.longitudinal_deviation = extractArray(path_history, '"longitudinal_deviation":[', max_vector_len);

    % Extract path intentions arrays
    path_intentions = extractNestedJson(jsonData, '"path_intentions":');

    structData.path_intentions.rel_time = extractArray(path_intentions, '"rel_time":[', max_vector_len);
    structData.path_intentions.lateral_deviation = extractArray(path_intentions, '"lateral_deviation":[', max_vector_len);
    structData.path_intentions.longitudinal_deviation = extractArray(path_intentions, '"longitudinal_deviation":[', max_vector_len);
end

function value = extractNumeric(jsonStr, fieldName)
    %#codegen
    % Extracts a numeric value from a JSON string.
    
    startIdx = strfind(jsonStr, fieldName);
    if isempty(startIdx)
        value = 0.0;
        return;
    end
    
    startIdx = startIdx + length(fieldName);
    endIdx = strfind(jsonStr(startIdx:end), ',') + startIdx - 2;
    if isempty(endIdx)
        endIdx = strfind(jsonStr(startIdx:end), '}') + startIdx - 2;
    end
    
    value = str2double(jsonStr(startIdx:endIdx));
end

function value = extractString(jsonStr, fieldName)
    %#codegen
    % Extracts a string value from a JSON string.
    
    startIdx = strfind(jsonStr, fieldName);
    if isempty(startIdx)
        value = ""; % Default
        return;
    end
    
    startIdx = startIdx + length(fieldName);
    endIdx = strfind(jsonStr(startIdx:end), '"') + startIdx - 2;
    
    value = jsonStr(startIdx:endIdx);
end

function json = extractNestedJson(jsonStr, fieldName)
    %#codegen
    % Extracts a sub JSON field from a JSON string.
    
    startIdx = strfind(jsonStr, fieldName);
    if isempty(startIdx)
        value = "{}";
        return;
    end
    
    startIdx = startIdx + length(fieldName);
    endIdx = strfind(jsonStr(startIdx:end), '}') + startIdx - 1;
    if isempty(endIdx)
        endIdx = strfind(jsonStr(startIdx:end), ',') + startIdx - 2;
    end
    
    json = jsonStr(startIdx:endIdx);
end

function values = extractArray(jsonStr, fieldName, max_vector_len)
    %#codegen
    % Extracts a numeric array from a JSON string while ensuring fixed-size compatibility.

    startIdx = strfind(jsonStr, fieldName);
    if isempty(startIdx)
        values = nan(max_vector_len, 1);
        return;
    end

    startIdx = startIdx + length(fieldName);
    endIdx = strfind(jsonStr(startIdx:end), ']') + startIdx - 2;

    % Convert extracted substring into numeric array
    numStr = jsonStr(startIdx:endIdx);
    strValues = str2double(split(numStr, ','));

    % Ensure fixed-size compatibility using direct indexing
    values = nan(max_vector_len, 1); % Preallocate 
    numEntries = min(length(strValues), max_vector_len);

    values(1:numEntries) = strValues(1:numEntries);
end