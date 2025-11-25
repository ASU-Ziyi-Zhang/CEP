%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Tyler Ard                       %%%
%%% Vehicle Mobility Systems Group  %%%
%%% tard(at)anl(dot)gov             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Notes
% File containing BSM message type
% BSM message structure?:
% https://www.tomesoftware.com/wp-content/uploads/2019/08/5-2019-B2V-Workshop-Detroit-Farid-Ahmed-Zaid-BSM-Messages.pdf

classdef BSM < matlab.System
    % BSM MATLAB System object representing vehicle motion data
    %
    % Latitude: Navigational latitude in meters (m).
    % Longitude: Navigational longitude in meters (m).
    % Heading: Navigational heading in radians, with 0 at the positive y-axis and increasing clockwise.
    % heading_rate_change: Time derivative of heading (radians/s).
    % Speed: Vehicle speed in meters per second (m/s).
    % Acceleration: Vehicle acceleration in meters per second squared (m/s²).
    % UTC Time: Coordinated Universal Time (UTC) timestamp for the current time.
    % error_flag: Char array indicating error status - error statuses defined by enum.
    %
    % Path history struct:
    % - Stores past vehicle positions with relative timestamps
    % - Used for analyzing previous motion trends
    % - Includes:
    %   - rel_time: Vector of past timestamps relative to current utc (negative valued) (s)
    %   - lateral_deviation: Lateral displacement relative to previous positions (m)
    %   - longitudinal_deviation: Forward displacement relative to previous positions (m)
    %
    % Path intentions struct:
    % - Stores predicted future positions based on vehicle motion
    % - Used for trajectory planning and movement forecasting
    % - Includes:
    %   - rel_time: Vector of future timestamps relative to current utc (s)
    %   - lateral_deviation: Projected lateral displacement (m)
    %   - longitudinal_deviation: Projected forward displacement (m)

    properties (Nontunable)
        % Required fields - check message() will error if these are not defined
        requiredKeys = {
            'current_utc_time', ...
            'latitude', ...
            'longitude', ...
            'speed', ...
            'acceleration', ...
            'heading', ...
            'heading_rate_change', ...
            'error_flag' ...
        };
            
        % Default values for optional fields - check message() will assign default values if these are not defined
        Defaults = struct( ...
            'elevation', 0.0, ...
            'position_accuracy', 1.5, ...
            'steering_angle', 0.0, ...
            'transmission_state', 'Drive', ...
            'braking_status', false, ...
            'turn_signal_status', 'Off', ...
            'length', 4.2, ...
            'width', 1.8, ...
            ... % Path history
            'times', [], ...
            'lateral_deviation', [], ...
            'longitudinal_deviation', [] ...
            ... % Path intentions
            'path_intentions', [], ...
            'rel_time', [], ...
            'lateral_deviation', [], ...
            'longitudinal_deviation', [] ...
        );

    end
        
    properties (DiscreteState)
        CurrentMessage % Stores the BSM message
    end
    
    methods (Access = protected)
        function out = getOutputDataTypeImpl(obj)
            out = 'BSM_Bus';
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = true;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = 1;
            dt = 'BSM_Bus';
            cp = false;
        end
        
        function resetImpl(obj)
            % Resets states (if needed)
            obj.CurrentMessage = struct();
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = 1;

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function setupImpl(obj)
            % Initializes the system object
            obj.CurrentMessage = struct( ...
                'current_utc_time', BSM.getUtcTime(), ...
                'latitude', 0.0, ...
                'longitude', 0.0, ...
                'speed', 0.0, ...
                'acceleration', 0.0, ...
                'heading', 0.0, ...
                'heading_rate_change', 0.0, ...
                'error_flag', "OK" ...
            );
        end
        
        function message = stepImpl(obj, latitude, longitude, speed, acceleration, heading, headingRateChange, times, positions)
            % Step function for generating a BSM message

            errorFlag = "OK"; % "OK" or "Error"
            
            message = struct( ...
                'current_utc_time', BSM.getUtcTime(), ...
                'latitude', latitude, ...
                'longitude', longitude, ...
                'speed', speed, ...
                'acceleration', acceleration, ...
                'heading', heading, ...
                'heading_rate_change', headingRateChange, ...
                'error_flag', errorFlag ...
            );

            % Include optional trajectory data            
            if nargin >= 8 && ~isempty(times) && ~isempty(positions) && all(isfinite(times))
                message.path_intentions = struct( ...
                    'rel_time', times, ... % Relative times [s]
                    'longitudinal_deviation', positions, ... % Longitudinal positions [m]
                    'lateral_deviation', nan(size(positions)) ... % Placeholder lateral deviation
                );
            end

            % Check and then return message
            message = obj.checkMessage(message);
            obj.CurrentMessage = message;
        end
    end
    
    methods (Static)
        function utcTime = getUtcTime()
            % Returns current UTC timestamp
            % utcTime = posixtime(datetime('now', 'TimeZone', 'UTC')); % TimeZone UTC not codegen compatible for some reason..
            utcTime = posixtime(datetime('now')); % TimeZone UTC not codegen compatible for some reason..
        end
    end
    
    methods (Access = private)
        function message = checkMessage(obj, message)
            % Validates required fields and fills optional values
            for i = 1:length(obj.requiredKeys)
                key = obj.requiredKeys{i};
                
                if ~isfield(message, key)
                    error('Missing required key: %s', key);
                end
            end
            
            % Fill defaults for missing optional fields
            defaultFields = fieldnames(obj.Defaults);
            
            for i = 1:length(defaultFields)
                key = defaultFields{i};
                
                if ~isfield(message, key)
                    message.(key) = obj.Defaults.(key);
                end
            end
        end
    end

end