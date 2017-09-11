function [points_n, r_n] = p_n(R,t,points_w)
    % This will convert world points to normalized scene points. It also
    % outputs normalized scene point distances since this is often used in
    % conjunction with normalized coordinates.
    %
    % Inputs:
    %   R - array; 3x3 rotation matrix
    %   t - array; 3x1 translation
    %   points_w - array; Nx2 array of points in world coordinates
    %
    % Outputs:
    %   points_n - array; Nx2 array of normalized scene points
    %   r_n - array; Nx1 array of normalized scene point distances
    
    % Get points in camera coordinates
    points_s = alg.p_s(R,t,points_w);    

    % Normalize coordinates
    x_n = points_s(:,1)./points_s(:,3);
    y_n = points_s(:,2)./points_s(:,3);
    r_n = sqrt(x_n.^2 + y_n.^2);   
    
    % Set output
    points_n = [x_n y_n];
end