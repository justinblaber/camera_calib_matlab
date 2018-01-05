function vals = array_interp(array,points,method)
    % Interpolates array at specified points.
    %
    % Inputs:
    %   array - array; MxN array
    %   points - array; Px2 array of points
    %   method - string; method of interpolation
    %
    % Outputs:
    %   vals - array; Px1 vector of interpolated array points
    
    I = griddedInterpolant({1:size(array,1),1:size(array,2)},array,method,'none');
    vals = I(points(:,2),points(:,1));
end