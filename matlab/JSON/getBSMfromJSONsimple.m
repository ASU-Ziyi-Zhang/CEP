%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function bsmMessage = getBSMfromJSONsimple(jsonData)
    % Converts a JSON message to BSM message struct.
    %
    % Outputs:
    %   jsonData - JSON formatted string.
    %
    % Inputs:
    %   bsmMessage - Struct containing the BSM data with fixed fields.
    
    bsmMessage = jsondecode(jsonData);
end