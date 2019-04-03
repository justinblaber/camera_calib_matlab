function sym_f = str2sym(str_f) %#ok<STOUT>
    % Converts input string to a symbolic function which has the form:
    %
    %   sym_f(arg1, arg2, ..., argN) = val
    %
    % Inputs:
    %   str_f - string;
    %
    % Outputs:
    %   sym_f - symbolic function;

    % Parse
    [~, args, val] = util.parse_str_f(str_f);

    % Declare symbolic arguments
    eval(['syms sym_f(' strjoin(args, ', ') ');']);

    % Assign symbolic function
    eval(['sym_f(' strjoin(args, ', ') ') = ' val ';']);
end
