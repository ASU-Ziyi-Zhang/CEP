%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function jsonData = setJSONfromBSMsimple(bsmMessage)
    % Converts a BSM message struct to JSON format.
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
    jsonData = jsonencode(bsmMessage);
end