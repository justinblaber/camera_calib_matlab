function gs_points = interp_array(array,points,method)
    % Interpolates array at specified points.
    %
    % Inputs:
    %   array - array; mxn array
    %   points - array; px2 array of points
    %   method - string; method of interpolation
    %
    % Outputs:
    %   gs_points - array; px1 vector of interpolated image points
    
    if exist('method','var')
        gs_points = interp2(1:size(array,2),1:size(array,1),array,points(:,1),points(:,2),method);
    else
        gs_points = interp2(1:size(array,2),1:size(array,1),array,points(:,1),points(:,2));
    end
end