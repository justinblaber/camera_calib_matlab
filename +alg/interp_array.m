function vals = interp_array(array, ps, method)
    % Interpolates array at specified points.
    %
    % Inputs:
    %   array - array; MxN array
    %   ps - array; Px2 array of points
    %   method - string; method of interpolation
    %
    % Outputs:
    %   vals - array; Px1 vector of interpolated array points

    I = griddedInterpolant({1:size(array, 1), 1:size(array, 2)}, array, method, 'none');
    vals = I(ps(:, 2), ps(:, 1));
end
