function vals = vol_interp(vol,points,method)
    % Interpolates volume at specified points.
    %
    % Inputs:
    %   vol - vol; MxNxP vol
    %   points - array; Rx3 array of points
    %   method - string; method of interpolation
    %
    % Outputs:
    %   vals - array; Rx1 vector of interpolated array points
      
    I = griddedInterpolant({1:size(vol,1),1:size(vol,2),1:size(vol,3)},vol,method,'none');
    vals = I(points(:,2),points(:,1),points(:,3));
end
