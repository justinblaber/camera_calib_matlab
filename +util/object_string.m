function str = object_string(obj)
    % Gets string for input object. Useful if you don't know what type the
    % object is.
    %
    % Inputs:
    %   obj - unknown; matlab object
    %
    % Outputs:
    %   str - char; string containing objects value

    % Get string
    str = matlab.unittest.diagnostics.ConstraintDiagnostic.getDisplayableString(obj);

    % Trim it
    str = strtrim(str);
end
