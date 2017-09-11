function vals = array_interp(array,points,method)
    % Interpolates array at specified points.
    %
    % Inputs:
    %   array - array; MxN array
    %   points - array; Px2 array of points
    %   method - string; optional method of interpolation
    %
    % Outputs:
    %   vals - array; Px1 vector of interpolated array points
    
    if exist('method','var')
        vals = interp2(1:size(array,2),1:size(array,1),array,points(:,1),points(:,2),method);
    else
        vals = interp2(1:size(array,2),1:size(array,1),array,points(:,1),points(:,2));
    end
end