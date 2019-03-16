function [name, args, val] = parse_str_f(str_f)
    % Parses input "symbolic function string" into:
    %
    %   name(args) = val
    %
    % Inputs:
    %   str_f - string;
    %
    % Outputs:
    %   name - string;
    %   args - cell;
    %   val - string;

    % Parse into function and its value
    str_f_split = strsplit(str_f, '=');

    if numel(str_f_split) ~= 2
        error('Input symbolic function string must have format of "name(args) = val"');
    end

    % Get function and val
    f = strtrim(str_f_split{1});
    val = strtrim(str_f_split{2});

    % Split function into name and args
    f_split = strsplit(f, {'(', ')'});

    if numel(f_split) ~= 3 || ~isempty(f_split{3})
        error('Input symbolic function string must have format of "name(args) = val"');
    end

    name = strtrim(f_split{1});
    args = strtrim(f_split{2});

    % Split args
    args = strtrim(strsplit(args, ','));
end
