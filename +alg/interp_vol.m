function vals = interp_vol(vol, ps, method)
    % Interpolates volume at specified points.
    %
    % Inputs:
    %   vol - vol; MxNxP vol
    %   ps - array; Rx3 array of points
    %   method - string; method of interpolation
    %
    % Outputs:
    %   vals - array; Rx1 vector of interpolated array points
      
    I = griddedInterpolant({1:size(vol,1),1:size(vol,2),1:size(vol,3)},vol,method,'none');
    vals = I(ps(:,2),ps(:,1),ps(:,3));
end
