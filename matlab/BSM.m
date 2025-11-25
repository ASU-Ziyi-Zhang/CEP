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
            ...
            % Path history struct:
            % - Stores past vehicle positions with relative timestamps
            % - Used for analyzing previous motion trends
            % - Includes:
            %   - rel_time: Vector of past timestamps relative to current utc (negative valued) (s)
            %   - lateral_deviation: Lateral displacement relative to previous positions (m)
            %   - longitudinal_deviation: Forward displacement relative to previous positions (m)
            'path_history', struct( ...
                'rel_time', [], ...
                'lateral_deviation', [], ...
                'longitudinal_deviation', [] ...
            ), ...
            ...
            % Path intentions struct:
            % - Stores predicted future positions based on vehicle motion
            % - Used for trajectory planning and movement forecasting
            % - Includes:
            %   - rel_time: Vector of future timestamps relative to current utc (s)
            %   - lateral_deviation: Projected lateral displacement (m)
            %   - longitudinal_deviation: Projected forward displacement (m)
            'path_intentions', struct( ...
                'rel_time', [], ...
                'lateral_deviation', [], ...
                'longitudinal_deviation', [] ...
            ) ...
        );
        
    properties (DiscreteState)
        CurrentMessage % Stores the BSM message
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initializes the system object
            obj.CurrentMessage = struct();
        end
        
        function message = stepImpl(obj, latitude, longitude, speed, acceleration, heading, headingRateChange, times, positions)
            % Step function for generating a BSM message

            errorFlag = BSM.ErrorEnum.OK
            
            message = struct( ...
                'current_utc_time', BSM.getUtcTime(), ...
                'latitude', latitude, ...
                'longitude', longitude, ...
                'speed', speed, ...
                'acceleration', acceleration, ...
                'heading', heading, ...
                'heading_rate_change', headingRateChange, ...
                'error_flag', char(errorFlag) ...
            );

            % Include optional trajectory data            
            if nargin >= 8 && ~isempty(times) && ~isempty(positions)
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
        
        function resetImpl(obj)
            % Resets states (if needed)
            obj.CurrentMessage = struct();
        end
    end
    
    methods (Static)
        function utcTime = getUtcTime()
            % Returns current UTC timestamp
            utcTime = posixtime(datetime('now', 'TimeZone', 'UTC'));
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

    enumeration
        % Enum class for representing various error statuses
        OK ("OK")
        DEFAULT_ERROR ("Error")

        % SENSOR_FAILURE ("Sensor Failure")
        % COMMUNICATION_ERROR ("Communication Error")
        % GPS_SIGNAL_LOST ("GPS Signal Lost")
    end

end