function data = read_data(file_path)
    % Reads "names" from data file. Data file must have format:
    %
    %   % comment
    %   name =
    %   array
    %   name = num
    %   name = string
    %   ...
    %
    % It returns everything as a struct with fields corresponding to the
    % name(s). If multiple names are found, then struct field will be a
    % cell array. It is assumed all "num" are those strings convertable to
    % a logical or double through str2num(); if not, they are assumed to be
    % a string.
    %
    % Inputs:
    %   file_path - string; path to data file to read from.
    %
    % Outputs:
    %   data - struct;

    % Check to make sure data file exists
    if exist(file_path, 'file') ~= 2
        error(['Data file: "' file_path '" does not exist.']);
    end

    % Initialize
    data = struct();

    % Go through line by line; treat all names as cell arrays for now (for
    % simplicity) and "uncell" names with single entries afterwards.
    f = fopen(file_path);
    line = fgetl(f);
    num_line = 1;
    in_array = false; % Gets set to true for lines in array
    while ischar(line)
        % Trim leading and trailing white spaces
        line = strtrim(line);

        % Make sure line isn't blank or a comment
        if ~isempty(line) && line(1) ~= '%'
            % Find equal sign(s)
            line_find = strfind(line, '=');
            if numel(line_find) > 0
                % One or more equal sign; get name and param
                name = strtrim(line(1:line_find(1)-1));
                param = strtrim(line(line_find(1)+1:end));

                % Initialize cell if name doesn't exist
                if ~isfield(data, name)
                    data.(name) = {};
                end

                % Param is either a num, string, or array
                if ~isempty(param)
                    % This is either a num or string
                    num = str2doubleorlogical(param);
                    if ~isempty(num)
                        % number
                        data.(name){end+1} = num;
                    else
                        % string
                        data.(name){end+1} = param;
                    end

                    % We are definitely no longer in an array
                    in_array = false;
                else
                    % This is an array; initialize it
                    data.(name){end+1} = [];
                    in_array = true;
                end
            else
                % No equal sign. We must be "in" an array for this to be
                % the case.
                if in_array
                    % Use name from previous iteration; attempt to
                    % concatenate array row.
                    row = str2doubleorlogical(line);

                    % Check to make sure line contains double or logical
                    if ~isempty(row)
                        % Check to make sure number of elements allows it
                        % to be vertically concatenated
                        if isempty(data.(name){end}) || ...
                           size(data.(name){end}, 2) == size(row, 2)
                            data.(name){end} = vertcat(data.(name){end}, ...
                                                       row);
                        else
                            error(['Failed to concatenate line: "' ...
                                    num2str(num_line) '" for array with ' ...
                                   'name: "' name '" and value: "' line ...
                                   '" because the number of elements do ' ...
                                   'not match the previous row.']);
                        end
                    else
                        error(['Failed to concatenate line: "' ...
                                num2str(num_line) '" for array with ' ...
                               'name: "' name '" and value: "' line ...
                               '" because it is not a double or logical ' ...
                               'number.']);
                    end
                else
                    error(['Unknown line: "' num2str(num_line) '" ' ...
                           'with value: "' line '". A line without an ' ...
                           '"=" is only valid for an array.']);
                end
            end
        end

        % Get next line
        line = fgetl(f);
        num_line = num_line+1;
    end
    fclose(f);

    % "Uncell" names with single entries.
    fields_data = fields(data);
    for i = 1:numel(fields_data)
        if numel(data.(fields_data{i})) == 1
            data.(fields_data{i}) = data.(fields_data{i}){1};
        end
    end
end

function num = str2doubleorlogical(str)
    num = str2num(str); %#ok<ST2NM>
    if ~isempty(num)
        % Note that if string is a function name, then this can cause
        % str2num to have issues, hence why the double and logical check
        % are done here
        if ~any(strcmp(class(num), {'double', 'logical'}))
            num = [];
        end
    end
end
