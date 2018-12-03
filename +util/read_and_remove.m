function [param, data] = read_and_remove(data, field)
    % Reads input "field" from struct, and then removes it from struct.
    %
    % Inputs:
    %   data - struct; struct containing "field"
    %   field - string; "field" in data
    %   
    % Outputs:
    %   param - unknown; value of data.(field)
    %   data - struct; input data with "field" removed.

    % Check to make sure struct contains "field"
    fields_data = fields(data);
    if ~any(strcmp(field, fields_data))
        error(['Field: "' field '" not found']);
    end
    
    % Get param
    param = data.(field);
    
    % Remove "field"
    data = rmfield(data, field);
end