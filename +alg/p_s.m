function points_s = p_s(R,t,points_w)
    % This will convert world points to scene points 
    %
    % Inputs:
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation
    %   points_w - array; Nx2 array of points in world coordinates
    %
    % Outputs:
    %   points_s - array; Nx3 array of points in camera coordinates
    
    % Apply xform to points to get them into camera coordinates    
    points_s = ([R(:,1) R(:,2) t] * [points_w ones(size(points_w,1),1)]')';
end
