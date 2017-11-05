function vals = vol_interp(vol,points,method)
    % Interpolates volume at specified points.
    %
    % Inputs:
    %   vol - vol; MxNxP vol
    %   points - array; Rx3 array of points
    %   method - string; optional method of interpolation
    %
    % Outputs:
    %   vals - array; Rx1 vector of interpolated array points
    
    if exist('method','var')
        vals = interp3(1:size(vol,2),1:size(vol,1),1:size(vol,3),vol,points(:,1),points(:,2),points(:,3),method);
    else
        vals = interp3(1:size(vol,2),1:size(vol,1),1:size(vol,3),vol,points(:,1),points(:,2),points(:,3));
    end
end