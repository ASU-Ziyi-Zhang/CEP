
% Open the model
modelName = 'main_sl';
open_system(modelName);

% Find all 'From Workspace' blocks
workspaceBlocks = find_system(modelName, 'BlockType', 'FromWorkspace');

% Loop through each and replace with 'Constant' blocks
iter_start = 1;
for i = iter_start:length(workspaceBlocks)
    % Get BSM related fields
    fields = fieldnames(bsm);

    fieldName = fields{i}; % Extract the field name
    fieldValue = bsm.(fieldName);
    
    % Get block to replace
    pos = get_param(workspaceBlocks{i}, 'Position'); % Store position

    % Add a Constant block
    constName = strcat(modelName, '/', fieldName);
    constValue = sprintf('bsm.%s', (fieldName)); % Access the corresponding value

    if isnumeric(fieldValue)
        add_block('simulink/Sources/Constant', constName, 'Position', pos);
    
        % Set a default value (modify as needed)
        set_param(constName, 'Value', constValue); % Adjust as required

    elseif ischar(fieldValue)
        add_block('simulink/String/String Constant', constName, 'Position', pos);
    
        % Set a default value (modify as needed)
        set_param(constName, 'String', constValue); % Adjust as required

    else
        error('Cannot recognize value of field')

    end
    
    % Remove the old block
    delete_block(workspaceBlocks{i}); 

    % Error check
    % assert(length(workspaceBlocks) == length(fields)-(iter_start-1))
end

% Save the model
save_system(modelName);