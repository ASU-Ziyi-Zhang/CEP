
function set_sfunction_mask_input_names(modelName, blockPath, portType)
    %%% % Example usage
    %%% set_sfunction_mask_input_names('YourModel', 'YourModel/SFunctionBlock', 'Inport');
    %%% set_sfunction_mask_input_names('YourModel', 'YourModel/SFunctionBlock', 'Outport')
    
    % Ensure model is open
    open_system(modelName);
    
    % Check if block exists
    if ~exist(blockPath, 'file')
        error('Block %s does not exist in the model.', blockPath);
    end

    % Create mask
    maskObj = Simulink.Mask.create(blockPath);
    
    % Define input names corresponding to BSM struct fields
    portNames = {
        'msg_type', 'id', 'current_utc_time', 'latitude', 'longitude', ...
        'rel_long_gap', 'rel_lat_gap', 'speed', 'acceleration', 'heading', ...
        'heading_rate_change', 'error_flag', 'elevation', 'position_accuracy', 'steering_angle', ...
        'transmission_state', 'braking_status', 'turn_signal_status', 'length', 'width', ...
        'path_history_n', 'path_history_time', 'path_history_rel_long_pos', 'path_history_rel_lat_pos', ...
        'path_intentions_n', 'path_intentions_time', 'path_intentions_rel_long_pos', 'path_intentions_rel_lat_pos' ...
    };

    % Assign input port names
    for i = 1:numel(portNames)
        maskObj.set('Ports', struct('Name', portNames{i}, 'PortType', portType, 'Number', i));
    end
    
    disp('✅ Mask applied with named input ports!');
end