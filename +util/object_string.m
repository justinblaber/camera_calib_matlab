function str = object_string(obj)
    % Gets string for input object. Useful if you don't know or don't want
    % to convert based on type.
    %
    % I Wrapped this in a class since this seems like a somewhat
    % undocumented function.
    %
    % Inputs:
    %   obj - unknown; matlab object
    %
    % Outputs:
    %   str - char; matlab string containing objects value

    % Get string
    str = matlab.unittest.diagnostics.ConstraintDiagnostic.getDisplayableString(obj);

    % Trim it
    str = strtrim(str);
end
