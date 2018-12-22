function write_data(data, file_path, suffix)
    % Writes data struct to file. Note that data struct fields must only
    % contain: string, double, logical, and cell arrays. If a field
    % contains a cell array, then the elements of the cell array must be a
    % string, double, or logical array (i.e. cell arrays cant be nested).
    %
    % Inputs:
    %   data - struct; struct to write to disk
    %   file_path - string; path to file to write to
    %   suffix - string; optional. suffix to append to names
    %
    % Outputs:
    %   None

    if ~exist('suffix', 'var')
        suffix = '';
    end

    % Iterate over fields
    fields_data = fields(data);
    for i = 1:numel(fields_data)
        % Grab field, param and name
        field = fields_data{i};
        param = data.(field);
        name = [field suffix];

        % Write param to file
        switch class(param)
            case 'cell'
                % If there's a cell, then only go 1 deep while iterating
                for j = 1:numel(param)
                    write_param(param{j}, name, file_path)
                end
            otherwise
                write_param(param, name, file_path)
        end
    end
end

function write_param(param, name, file_path)
    switch class(param)
        case 'char'
            util.write_string(param, name, file_path);
        case {'double', 'logical'}
            % If logical, it will store a true as 1 and false as 0
            if isscalar(param)
                util.write_num(param, name, file_path);
            else
                util.write_array(param, name, file_path);
            end
        otherwise
            error(['A param: "' name '" was ' ...
                   'found in calibration config that has ' ...
                   'unsupported type of: "' class(param) '".']);
    end
end
