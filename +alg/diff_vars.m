function sym_diff = diff_vars(sym, args)
    % Computes derivative(s) of symbolic function
    %   
    % Inputs:
    %   sym - symbolic function
    %   args - cell or string; cell of strings or a string containing 
    %       variable name(s) to differentiate
    % 
    % Outputs:
    %   sym_diff - symbolic function; derivative(s) of sym
    
    % If input is a string, just convert it to a cell
    if ischar(args)
        args = {args};
    end
    
    % Compute derivatives
    for i = 1:numel(args)
        if ~exist('sym_diff','var')
            sym_diff = diff(sym, args{i});
        else
            sym_diff = vertcat(sym_diff, diff(sym, args{i})); %#ok<AGROW>
        end
    end
end