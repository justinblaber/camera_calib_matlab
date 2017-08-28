function gs_points = array_interp(array,points,method)
    % Interpolates array at specified points.
    %
    % Inputs:
    %   array - array; MxN array
    %   points - array; Px2 array of points
    %   method - string; method of interpolation
    %
    % Outputs:
    %   gs_points - array; Px1 vector of interpolated array points
    
    if exist('method','var')
        gs_points = interp2(1:size(array,2),1:size(array,1),array,points(:,1),points(:,2),method);
    else
        gs_points = interp2(1:size(array,2),1:size(array,1),array,points(:,1),points(:,2));
    end
end