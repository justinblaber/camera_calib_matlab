function str_f = sym2str(sym_f)
    % Converts input symbolic function to a string which has the form:
    %
    %   sym_f(arg1, arg2, ..., argN) = val
    %
    % Inputs:
    %   sym_f - symbolic function;
    %
    % Outputs:
    %   str_f - string;

    if ~isa(sym_f, 'sym')
        error('Input is not a symbolic function');
    end

    % Get string
    str_f = util.object_string(sym_f);

    % Remove multi-line stuff
    str_f = strrep(str_f, newline, '');

    % Parse
    [~, args, val] = util.parse_str_f(str_f);

    % Join
    str_f = ['sym_f(' strjoin(args, ', ') ') = ' val];
end
